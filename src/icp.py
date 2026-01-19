import numpy as np


def find_correspondences(source, target, max_distance=None):
    """
    Find nearest neighbor in target for each point in source.

    Returns:
        matched_source: (K, 2) array - source points that have matches
        matched_target: (K, 2) array - corresponding target points
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

    return source[src_indices], target[tgt_indices], np.array(distances)


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
    Use SVD to find optimal R and t.

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


def icp(source, target, max_iterations=50, tolerance=1e-4, max_distance=0.5):
    """
    Align source point cloud to target using ICP.

    Args:
        source: (N, 2) array - points to align
        target: (M, 2) array - reference points
        max_iterations: max number of iterations
        tolerance: convergence threshold (radians for rotation, meters for translation)
        max_distance: reject correspondences further than this (meters)

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

    for iteration in range(max_iterations):
        # Step 1: Find correspondences (with outlier rejection)
        matched_source, matched_target, dists = find_correspondences(
            source_current, target, max_distance=max_distance
        )

        # Step 2: Compute centroids
        centroid_source = matched_source.mean(axis=0)
        centroid_target = matched_target.mean(axis=0)

        # Step 3: Center points
        centered_source = matched_source - centroid_source
        centered_target = matched_target - centroid_target

        # Step 4: Compute cross-covariance and optimal R, t
        H = compute_cross_covariance(centered_source, centered_target)
        R, t = compute_rotation_translation(H, centroid_source, centroid_target)

        # Step 5: Apply transformation to current source
        source_current = (R @ source_current.T).T + t

        # Step 6: Accumulate total transformation
        total_t = R @ total_t + t
        total_R = R @ total_R

        # Step 7: Check convergence
        # 2D rotation: R[0,0] = cos(θ)
        angle_change = np.arccos(np.clip(R[0, 0], -1, 1))
        translation_change = np.linalg.norm(t)

        if angle_change < tolerance and translation_change < tolerance:
            return total_R, total_t, source_current, True

    return total_R, total_t, source_current, False
