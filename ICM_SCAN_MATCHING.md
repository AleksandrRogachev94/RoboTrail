# ICP Scan Matching: Understanding from First Principles

This guide teaches you the ICP (Iterative Closest Point) algorithm by building intuition for the underlying mathematics. By the end, you'll understand not just _what_ to implement, but _why_ each step works.

---

## Part 1: The Problem We're Solving

Imagine you take a laser scan of a room, drive forward 20cm, and take another scan. You _think_ you moved 20cm, but wheel slip and surface variations mean you might have actually moved 18cm or 22cm. How do you figure out your _true_ motion?

**The insight**: If both scans see the same walls, you can compare them. The transformation (rotation + translation) that makes the second scan "land on top of" the first scan **is** your actual motion.

```
Scan A (before):           Scan B (after):           Aligned:
    • • • •                    • • • •                 • • • •
    •       •                  •       •               ••     ••
    •       •      ──────►     •       •    ──────►    ••     ••
    •       •                  •       •               ••     ••
    • • • •                    • • • •                 • • • •
   (fixed)                   (needs alignment)        (overlapped)
```

The challenge: we don't know which point in Scan B corresponds to which point in Scan A. This is the **correspondence problem**, and it's what makes scan matching tricky.

---

## Part 2: Rigid Transformations in 2D

Before tackling the full algorithm, let's make sure we understand how to describe motion mathematically.

### What Is a Rigid Transformation?

A **rigid transformation** preserves distances and angles—it's pure rotation and translation, no stretching or shearing. When your robot moves, it undergoes a rigid transformation.

Any rigid transformation in 2D can be described by:

- A rotation angle **θ**
- A translation vector **t = [tx, ty]ᵀ**

To transform a point **p** to a new position **p'**:

```
p' = R(θ) · p + t
```

### The Rotation Matrix

Why do we use matrices for rotation? Let's derive it from scratch.

Consider a point **p = [x, y]ᵀ**. We want to rotate it by angle θ around the origin. Using basic trigonometry:

If the original point is at angle φ and distance r from origin:

- x = r·cos(φ)
- y = r·sin(φ)

After rotating by θ, the new angle is (φ + θ), but distance r stays the same:

- x' = r·cos(φ + θ)
- y' = r·sin(φ + θ)

Using the angle addition formulas:

- cos(φ + θ) = cos(φ)cos(θ) - sin(φ)sin(θ)
- sin(φ + θ) = sin(φ)cos(θ) + cos(φ)sin(θ)

Substituting x = r·cos(φ) and y = r·sin(φ):

- x' = x·cos(θ) - y·sin(θ)
- y' = x·sin(θ) + y·cos(θ)

Writing this as a matrix equation:

```
┌ x' ┐   ┌ cos(θ)  -sin(θ) ┐   ┌ x ┐
│    │ = │                 │ · │   │
└ y' ┘   └ sin(θ)   cos(θ) ┘   └ y ┘
```

That's our rotation matrix **R(θ)**.

### Key Properties of Rotation Matrices

**Property 1: Orthogonality (Rᵀ = R⁻¹)**

Rotation doesn't stretch anything—it preserves lengths. Mathematically, this means:

```
RᵀR = I  (identity matrix)
```

This is incredibly useful because inverting a rotation is just transposing it—no expensive matrix inversion needed. To "undo" a rotation by θ, just apply R(-θ) = Rᵀ.

**Property 2: Determinant = 1**

det(R) = cos²(θ) + sin²(θ) = 1

This confirms no stretching or reflection. A reflection would have det = -1.

**Property 3: Rotations Compose by Addition**

R(θ₁) · R(θ₂) = R(θ₁ + θ₂)

You can verify this by multiplying out the matrices and using trig identities.

---

## Part 3: The ICP Algorithm Overview

ICP solves the alignment problem through iteration:

```
1. GUESS: Start with some initial transformation (often from odometry)
2. MATCH: For each point in scan B, find the closest point in scan A
3. SOLVE: Find the best transformation that aligns the matched pairs
4. APPLY: Transform scan B by this transformation
5. REPEAT: Go back to step 2, using the newly transformed B

Stop when the transformation becomes negligibly small (convergence).
```

Why iterate? Because step 2 (finding correspondences) depends on how well aligned the scans are. With a bad initial alignment, you get wrong correspondences. But after aligning a little, correspondences improve, which allows better alignment, and so on.

---

## Part 4: The Alignment Problem (Given Correspondences)

Let's temporarily assume someone gave us correct correspondences—matched pairs of points:

- (b₁, a₁), (b₂, a₂), ..., (bₖ, aₖ)

We want to find R and t that minimize the total squared error:

```
E(R, t) = Σᵢ || aᵢ - (R·bᵢ + t) ||²
```

This is a **least squares** problem. Let's understand why we use squared errors and how to solve it.

### Why Squared Errors?

Consider three candidate transformations with these residual errors for 4 point pairs:

| Transform | Errors               | Sum of | errors |     | Sum of errors² |
| --------- | -------------------- | ------ | ------ | --- | -------------- |
| T₁        | [1, 1, 1, 1]         | 4      | 4      |
| T₂        | [0, 0, 0, 4]         | 4      | 16     |
| T₃        | [0.5, 0.5, 0.5, 0.5] | 2      | 1      |

**Observation 1**: Sum of absolute errors treats T₁ and T₂ equally, but T₂ is worse—it has one terrible match while T₁ is consistently decent.

**Observation 2**: Squaring penalizes large errors more heavily. T₂ gets punished for having a 4-unit error.

**Observation 3**: The function E = Σ(error)² is smooth and differentiable everywhere, making it amenable to calculus-based optimization. Absolute values have a kink at zero.

**Observation 4**: If errors are Gaussian-distributed (which sensor noise often is), minimizing squared error gives the Maximum Likelihood Estimate—the most probable solution.

### Decoupling Rotation and Translation

Here's a key insight that simplifies our problem: **we can solve for rotation first, then translation falls out trivially**.

Define the centroids (centers of mass):

```
μ_a = (1/K) Σᵢ aᵢ    (centroid of the A points)
μ_b = (1/K) Σᵢ bᵢ    (centroid of the B points)
```

Define centered points:

```
a'ᵢ = aᵢ - μ_a
b'ᵢ = bᵢ - μ_b
```

**Theorem**: The optimal rotation minimizes the error between centered points, and the optimal translation is:

```
t* = μ_a - R* · μ_b
```

**Why does this work?** Think about it geometrically. After optimal alignment:

- The centroid of transformed B should land on the centroid of A
- R·μ_b + t = μ_a
- Therefore t = μ_a - R·μ_b

By centering, we remove the translation component entirely from the rotation optimization. Now we just need to find R that minimizes:

```
Σᵢ || a'ᵢ - R·b'ᵢ ||²
```

---

## Part 5: Understanding SVD (The Heart of the Solution)

Singular Value Decomposition is one of the most powerful tools in linear algebra. Let's build intuition for what it does.

### What Does a Matrix DO?

Any matrix M represents a linear transformation. When you multiply M by a vector, you're transforming that vector.

Consider what happens when you apply a 2×2 matrix to the unit circle (all vectors of length 1):

```
Unit circle → [Matrix M] → Ellipse
```

The circle gets stretched into an ellipse. This is true for _any_ matrix (except the zero matrix).

### The Geometry of SVD

SVD says any matrix M can be decomposed as:

```
M = U · Σ · Vᵀ
```

What this means geometrically:

1. **Vᵀ** rotates/reflects the input
2. **Σ** stretches along the coordinate axes
3. **U** rotates/reflects the output

```
Input      Vᵀ           Σ              U           Output
  ○   ──────────►  ○  ──────────►  ⬭  ──────────►   ⬭
circle           rotated        ellipse         rotated
                 circle         (axis-aligned)  ellipse
```

**The singular values** (diagonal of Σ) are the scaling factors—they tell you how much stretching happens in each direction.

**The columns of V** are the input directions that get mapped to the principal axes.

**The columns of U** are the output principal axes.

### Computing SVD: What's Happening?

Let's think about the 2×2 case for our matrix H (the cross-covariance matrix we'll build).

The singular values σ₁, σ₂ are the square roots of the eigenvalues of HᵀH (or HHᵀ).

Wait, what are eigenvalues? If you've forgotten:

**Eigenvalue refresher**: For a square matrix A, an eigenvector v satisfies:

```
A·v = λ·v
```

Multiplying by A just scales v by factor λ (the eigenvalue). The vector points in the same direction.

For a symmetric matrix (like HᵀH), the eigenvectors are orthogonal to each other—they form a natural coordinate system. The eigenvalues tell you how much the matrix "stretches" in each eigenvector direction.

**Connection to SVD**:

- Columns of V are eigenvectors of HᵀH
- Columns of U are eigenvectors of HHᵀ
- Singular values = √(eigenvalues of HᵀH)

In practice, you call `numpy.linalg.svd(H)` and it handles the numerical details.

### Why SVD Solves Our Rotation Problem

Here's the beautiful result. Given our centered points, define:

```
H = Σᵢ b'ᵢ · a'ᵢᵀ
```

This is called the **cross-covariance matrix**. Each term is an outer product:

```
b'ᵢ · a'ᵢᵀ = ┌ b'x ┐            ┌ b'x·a'x  b'x·a'y ┐
             │     │ · [a'x a'y] = │                 │
             └ b'y ┘            └ b'y·a'x  b'y·a'y ┘
```

Summing over all point pairs gives H.

**What does H capture?** It encodes the correlation between the two point sets. If b and a tend to point in similar directions, certain entries of H will be large and positive.

**The SVD magic**: Decompose H = UΣVᵀ. Then:

```
R* = V · Uᵀ
```

### Why Does R = VUᵀ Work?

Let me give you the intuition (not a formal proof, but enough to understand).

Remember what SVD does: H = UΣVᵀ means H takes vectors, rotates by Vᵀ, scales by Σ, then rotates by U.

We want R such that:

```
Σᵢ || a'ᵢ - R·b'ᵢ ||²  is minimized
```

Expanding and simplifying (using properties of traces and orthogonal matrices), this becomes equivalent to:

```
Maximize: trace(RᵀH)
```

The trace of a matrix is the sum of its diagonal elements—it represents a sort of "alignment score" between R and H.

Now, RᵀH = Rᵀ(UΣVᵀ). We want to maximize trace(RᵀUΣVᵀ).

Since Σ is diagonal (with positive entries), and we want to maximize the trace, we need RᵀU = V, which means R = VUᵀ.

**Physical intuition**: U captures how the A points are oriented. V captures how the B points are oriented. VUᵀ is the rotation that maps B's orientation to A's orientation.

### The Reflection Edge Case

SVD gives orthogonal matrices U and V, but orthogonal matrices can be rotations (det = +1) or reflections (det = -1).

If det(VUᵀ) = -1, we got a reflection, not a rotation. Robots don't reflect! To fix this:

```
If det(V · Uᵀ) < 0:
    Flip the sign of the last column of V
    Recompute R = V · Uᵀ
```

This gives the best rotation (not reflection).

---

## Part 6: The Correspondence Problem

We've solved alignment assuming known correspondences. But how do we find correspondences?

### Nearest Neighbor Matching

The simplest approach: for each point bᵢ in scan B, find the closest point in scan A.

```
For each bᵢ:
    corresponding[i] = argmin_j || bᵢ - aⱼ ||
```

This requires computing distances from each B point to every A point—O(N×M) operations.

**When does this work?** When scans are approximately aligned. If B is rotated 90° from A, nearest neighbors will be completely wrong.

**The bootstrap**: Use odometry as initial guess. Odometry might be off by a few cm/degrees, but it's not off by 90°. This gets us close enough for nearest neighbor to work.

### Improving Correspondences with Iteration

Even if initial correspondences are imperfect, they're "approximately right" if the initial guess is decent. Aligning with imperfect correspondences still moves scans closer together.

After alignment, correspondences improve. After a few iterations, they stabilize at the correct matching.

This is why ICP is **iterative**—each iteration improves both alignment and correspondences.

### Outlier Rejection

Some correspondences are just wrong:

- Points that don't have true matches (one scan sees a door, other doesn't)
- Sensor noise creating spurious points
- Moving objects (person walked by during one scan)

Bad correspondences corrupt the alignment. Common fixes:

**Distance threshold**: If |b - closest_a| > threshold, reject the pair. A wall hasn't moved 2 meters between scans.

**Percentage-based**: Keep only the 80% of pairs with smallest distances. Outliers tend to have larger distances.

**Statistical**: Compute mean and std of distances, reject pairs > 2 std from mean.

---

## Part 7: Point-to-Line ICP (An Improvement)

Basic ICP minimizes point-to-point distances. But think about a wall:

```
Scan A:       · · · · · · ·    (points along wall)
Scan B:         · · · · · ·    (shifted along wall)
```

Point-to-point ICP wants each B point to exactly match some A point. But if B is shifted along the wall, points might match to wrong neighbors, causing jitter.

**Point-to-line insight**: Points on a wall can slide along the wall freely—only the perpendicular distance matters.

Instead of minimizing || bᵢ - aⱼ ||², minimize:

```
( (bᵢ - aⱼ) · nⱼ )²
```

where nⱼ is the **surface normal** at aⱼ (perpendicular to the wall).

**How to get normals?** Fit a line to aⱼ and its neighbors, then take the perpendicular.

Point-to-line converges much faster because it doesn't fight against the "slide along surface" ambiguity.

---

## Part 8: Convergence and Local Minima

### Why Does ICP Converge?

Each iteration consists of:

1. Finding nearest neighbors (correspondences)
2. Computing optimal alignment

Step 2 is a closed-form optimization—it always finds the global minimum for the _current_ correspondences.

After aligning, the total error either:

- Decreases (we found a better alignment), or
- Stays the same (we're at a fixed point)

Since error is bounded below by zero and decreases each iteration, it must converge.

### Local Minima: The Catch

ICP converges to a **local** minimum, not necessarily the global minimum. If the initial guess is bad, it might converge to a wrong alignment.

```
True alignment:   Scans overlap perfectly, error ≈ 0
Local minimum:    Scans partially overlap, error > 0 but stable
```

**Mitigation strategies**:

- Good initial guess (odometry)
- Multiple restarts with different initial guesses
- Branch-and-bound or other global methods (expensive, usually unnecessary)

For your robot with odometry giving a reasonable initial guess, local minima are rarely a problem.

---

## Part 9: Putting It All Together

Here's the complete picture:

```
┌────────────────────────────────────────────────────────────────┐
│  INPUT: Scan A (reference), Scan B (to align), initial guess  │
└────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌────────────────────────────────────────────────────────────────┐
│  ITERATION LOOP:                                               │
│                                                                │
│  1. Apply current transform to B                               │
│                                                                │
│  2. For each point in B, find nearest neighbor in A            │
│     → Create correspondence pairs                              │
│     → Reject outliers (distance threshold)                     │
│                                                                │
│  3. Compute centroids μ_a, μ_b                                 │
│  4. Center the points: a'ᵢ = aᵢ - μ_a, b'ᵢ = bᵢ - μ_b         │
│  5. Build cross-covariance: H = Σᵢ b'ᵢ · a'ᵢᵀ                 │
│  6. SVD: H = UΣVᵀ                                              │
│  7. Rotation: R = VUᵀ (with reflection check)                  │
│  8. Translation: t = μ_a - R · μ_b                             │
│                                                                │
│  9. Update total transformation                                │
│  10. Check convergence (is transform negligible?)              │
│      If no → repeat from step 1                                │
│      If yes → exit loop                                        │
└────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌────────────────────────────────────────────────────────────────┐
│  OUTPUT: Final (R, t) that aligns B to A                       │
│          This represents how the robot actually moved          │
└────────────────────────────────────────────────────────────────┘
```

---

## Part 10: Connecting to Your Robot

### Scan in Robot Frame

Your ToF sensor gives readings in polar coordinates relative to the robot:

- Angle: servo position (0° to 180°)
- Distance: ToF reading

Convert to Cartesian (still in robot frame):

```
x = distance · cos(angle)
y = distance · sin(angle)
```

### What ICP Gives You

ICP returns the transformation from scan B to scan A. If:

- Scan A was taken at pose (x₁, y₁, θ₁)
- Scan B was taken at pose (x₂, y₂, θ₂)

Then ICP's (R, t) represents the motion from pose 1 to pose 2.

### Comparison with Odometry

Odometry says you moved (Δx_odo, Δy_odo, Δθ_odo).
ICP says you actually moved (R_icp, t_icp).

The difference is odometry error. You can:

1. Trust ICP completely (replace odometry estimate)
2. Fuse them (use EKF to combine both estimates)

For your slow-moving robot with decent odometry, option 1 often works fine.

---

## Part 11: Testing Strategy

### Test 1: Identity

```
A = B (identical scans)
Expected: R = I, t = [0, 0]
```

If this fails, you have a bug in the basic algorithm.

### Test 2: Pure Translation

```
B = A + [0.1, 0]  (shifted 10cm right)
Expected: t = [0.1, 0], R = I
```

Tests that translation recovery works.

### Test 3: Pure Rotation

```
B = R(15°) · A  (rotated 15°)
Expected: R = R(15°), t = [0, 0]
```

Tests that rotation recovery works. Note: rotation around origin, so t should be zero.

### Test 4: Combined Transform

```
B = R(10°) · A + [0.05, 0.03]
Expected: recover both correctly
```

### Test 5: Gaussian Noise

Add σ = 2cm noise to B (matching your VL53L1X).

```
Expected: Close to true transform, not exact
```

### Test 6: Partial Overlap

Remove 30% of points from each scan (simulating different fields of view).

```
Expected: Still converges (tests outlier handling)
```

### Test 7: Bad Initial Guess

Start with deliberately wrong initial R, t.

```
Expected: Converges if initial error is "small enough"
         May fail if initial error is too large
```

---

## Summary

The key insights to remember:

1. **ICP alternates between correspondence and alignment** because each depends on the other

2. **Alignment is a least squares problem** that minimizes squared error between matched point pairs

3. **Centering decouples rotation from translation**—solve rotation first, translation follows trivially

4. **SVD of the cross-covariance matrix gives the optimal rotation** because it captures the orientational relationship between point sets

5. **Convergence is guaranteed** (to a local minimum) because each step can only decrease error

6. **Good initialization matters** to avoid local minima—use odometry

7. **Outlier rejection prevents bad correspondences** from corrupting the solution

With this understanding, you're ready to implement ICP. Start with the simplest version (point-to-point, distance threshold for outliers), verify with synthetic tests, then add refinements as needed.
