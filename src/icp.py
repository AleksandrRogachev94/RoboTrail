import numpy as np


def find_correspondences(source, target, max_distance=None):
    """
    Find nearest neighbor in target for each point in source.

    Returns:
        matched_source: (K, 2) array - source points that have matches
        matched_target: (K, 2) array - corresponding target points
        matched_indices: (K,) array - indices into target for each match
        distances: (K,) array - distances for each match
    """
    src_indices = []
    tgt_indices = []
    distances = []

    for i, pt in enumerate(source):
        dists = np.linalg.norm(target - pt, axis=1)
        j = np.argmin(dists)
        min_dist = dists[j]

        if max_distance is None or min_dist <= max_distance:
            src_indices.append(i)
            tgt_indices.append(j)
            distances.append(min_dist)

    return (
        source[src_indices],
        target[tgt_indices],
        np.array(tgt_indices),
        np.array(distances),
    )


def estimate_normals(map_points, k=6):
    """
    Estimate surface normals at each map point using PCA on k nearest neighbors.

    For each map point, we look at its k neighbors and fit a line through them.
    The normal is perpendicular to that line (the direction of least variance).

    Args:
        map_points: (M, 2) array of map points
        k: number of neighbors to use for normal estimation

    Returns:
        normals: (M, 2) array of unit normal vectors
        reliable: (M,) boolean array - True if normal is trustworthy
    """
    M = len(map_points)
    normals = np.zeros((M, 2))
    reliable = np.zeros(M, dtype=bool)

    # Need at least k+1 points total to find k neighbors
    if M < k + 1:
        return normals, reliable

    for i in range(M):
        # Find k nearest neighbors (excluding self)
        dists = np.linalg.norm(map_points - map_points[i], axis=1)
        dists[i] = np.inf  # exclude self
        neighbor_idx = np.argpartition(dists, k)[:k]
        neighbors = map_points[neighbor_idx]

        # PCA: fit a line through the neighbors
        centroid = neighbors.mean(axis=0)
        centered = neighbors - centroid
        C = centered.T @ centered  # 2x2 covariance matrix

        # SVD: smallest singular value → normal direction
        _, S, Vt = np.linalg.svd(C)
        normal = Vt[-1]  # last row = direction of least variance = normal

        # Reliability check: the surface is "flat" (linear) if the ratio of
        # singular values is large. If S[0] ≈ S[1], it's a corner or cluster.
        # Also check that neighbors are close enough to form a coherent surface.
        if S[0] > 1e-8:
            linearity = 1.0 - S[1] / S[0]  # 0=corner/cluster, 1=perfect line
            max_neighbor_dist = dists[neighbor_idx].max()
            reliable[i] = linearity > 0.5 and max_neighbor_dist < 30.0
        else:
            reliable[i] = False

        normals[i] = normal

    return normals, reliable


def compute_cross_covariance(centered_source, centered_target):
    """
    Build cross-covariance matrix H = Σᵢ bᵢ · aᵢᵀ

    Args:
        centered_source: (N, 2) array - centered B points
        centered_target: (N, 2) array - centered A points (matched pairs)

    Returns:
        H: (2, 2) cross-covariance matrix
    """
    return np.dot(centered_source.T, centered_target)


def compute_rotation_translation(H, centroid_source, centroid_target):
    """
    Use SVD to find optimal R and t (point-to-point step).

    Args:
        H: (2, 2) cross-covariance matrix
        centroid_source: (2,) centroid of source points
        centroid_target: (2,) centroid of target points

    Returns:
        R: (2, 2) rotation matrix
        t: (2,) translation vector
    """
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1  # Flip last row of Vt (= last column of V)
        R = Vt.T @ U.T
    t = centroid_target - R @ centroid_source
    return R, t


def compute_point_to_line_step(matched_source, matched_target, normals):
    """
    Solve for the rigid transform [dx, dy, dθ] that minimizes point-to-line error.

    For each matched pair (s_i, m_i) with surface normal n_i, the error is:
        e_i = dot(s_i + [dx, dy] + dθ*perp(s_i) - m_i, n_i)
    where perp([x,y]) = [-y, x] (90° rotation for small-angle linearization).

    This gives a linear system A·x = b where x = [dx, dy, dθ].

    Args:
        matched_source: (N, 2) source points (already transformed to current estimate)
        matched_target: (N, 2) corresponding target/map points
        normals: (N, 2) surface normals at each target point

    Returns:
        R: (2, 2) rotation matrix for this step
        t: (2,) translation vector for this step
        success: bool - False if system is degenerate
    """
    N = len(matched_source)
    if N < 3:
        return np.eye(2), np.zeros(2), False

    # Build linear system: A @ [dx, dy, dθ] = b
    # Row i: [nx_i, ny_i, -nx_i*sy_i + ny_i*sx_i] · [dx, dy, dθ] = dot(m_i - s_i, n_i)
    sx = matched_source[:, 0]
    sy = matched_source[:, 1]
    nx = normals[:, 0]
    ny = normals[:, 1]

    A = np.column_stack([nx, ny, -nx * sy + ny * sx])
    b = np.sum((matched_target - matched_source) * normals, axis=1)

    # Solve least squares
    result, residuals, rank, sv = np.linalg.lstsq(A, b, rcond=None)

    # Degenerate if rank < 3 (e.g., all normals point the same direction)
    if rank < 3:
        return np.eye(2), np.zeros(2), False

    dx, dy, dtheta = result

    # Clamp to prevent wild steps (especially rotation)
    max_translation = 5.0  # cm per iteration (conservative)
    max_rotation = np.radians(5.0)  # 5 degrees per iteration
    dx = np.clip(dx, -max_translation, max_translation)
    dy = np.clip(dy, -max_translation, max_translation)
    dtheta = np.clip(dtheta, -max_rotation, max_rotation)

    # Convert [dx, dy, dθ] to R, t
    c, s = np.cos(dtheta), np.sin(dtheta)
    R = np.array([[c, -s], [s, c]])
    t = np.array([dx, dy])

    return R, t, True


def icp(
    source,
    target,
    max_iterations=50,
    tolerance=1e-4,
    max_distance=0.5,
    use_point_to_line=True,
):
    """
    Align source point cloud to target using ICP.

    Uses a hybrid approach:
    - Point-to-line (point-to-plane in 2D) where surface normals are reliable
    - Point-to-point (SVD) as fallback when normals are unreliable

    Point-to-line is much more accurate for sparse scans because it constrains
    the scan point to lie ON the wall surface, not just near a specific map dot.

    Args:
        source: (N, 2) array - points to align (scan in world frame)
        target: (M, 2) array - reference points (map)
        max_iterations: max number of iterations
        tolerance: convergence threshold (radians for rotation, meters for translation)
        max_distance: reject correspondences further than this (meters/cm, same units as input)
        use_point_to_line: if True, use hybrid point-to-line + point-to-point

    Returns:
        total_R: (2, 2) total rotation matrix
        total_t: (2,) total translation vector
        transformed_source: aligned points
        converged: boolean - True if converged before max_iterations
    """
    # Initialize totals
    total_R = np.eye(2)
    total_t = np.zeros(2)
    source_current = source.copy()

    # Pre-compute normals for all map points (done once, not per iteration)
    if use_point_to_line and len(target) >= 7:
        map_normals, map_reliable = estimate_normals(target, k=6)
    else:
        map_normals = None
        map_reliable = None

    for iteration in range(max_iterations):
        # Step 1: Find correspondences (with outlier rejection)
        matched_source, matched_target, tgt_indices, dists = find_correspondences(
            source_current, target, max_distance=max_distance
        )

        if len(matched_source) < 3:
            break

        # Require at least 40% of source points to have matches
        # Otherwise ICP is fitting to too few points and is unreliable
        match_ratio = len(matched_source) / len(source)
        if match_ratio < 0.4:
            break

        # Step 2: Decide which correspondences have reliable normals
        if map_normals is not None and len(tgt_indices) > 0:
            corr_normals = map_normals[tgt_indices]
            corr_reliable = map_reliable[tgt_indices]

            n_reliable = corr_reliable.sum()

            if n_reliable >= 3:
                # --- Hybrid step ---
                # Use point-to-line for reliable correspondences
                ptl_src = matched_source[corr_reliable]
                ptl_tgt = matched_target[corr_reliable]
                ptl_normals = corr_normals[corr_reliable]

                R, t, success = compute_point_to_line_step(
                    ptl_src, ptl_tgt, ptl_normals
                )

                if not success:
                    # Fall back to point-to-point for this iteration
                    R, t = _point_to_point_step(matched_source, matched_target)
            else:
                # Not enough reliable normals → full point-to-point
                R, t = _point_to_point_step(matched_source, matched_target)
        else:
            # Point-to-line disabled or not enough map points
            R, t = _point_to_point_step(matched_source, matched_target)

        # Step 3: Apply transformation to current source
        source_current = (R @ source_current.T).T + t

        # Step 4: Accumulate total transformation
        total_t = R @ total_t + t
        total_R = R @ total_R

        # Step 5: Check convergence
        angle_change = np.arccos(np.clip(R[0, 0], -1, 1))
        translation_change = np.linalg.norm(t)

        if angle_change < tolerance and translation_change < tolerance:
            return total_R, total_t, source_current, True

    return total_R, total_t, source_current, False


def _point_to_point_step(matched_source, matched_target):
    """Standard point-to-point SVD step (used as fallback)."""
    centroid_source = matched_source.mean(axis=0)
    centroid_target = matched_target.mean(axis=0)
    centered_source = matched_source - centroid_source
    centered_target = matched_target - centroid_target
    H = compute_cross_covariance(centered_source, centered_target)
    return compute_rotation_translation(H, centroid_source, centroid_target)
