# Graph SLAM Guide

This guide teaches you pose graph SLAM, building on the scan-matching foundation from [ICM_SCAN_MATCHING.md](ICM_SCAN_MATCHING.md).

> [!IMPORTANT] > **Prerequisite:** You should be comfortable with ICP scan matching (Stage 4) before tackling this. Graph SLAM builds on top of ICP, not as a replacement.

---

## Part 1: Why a Pose Graph?

The current SLAM system works like this:

```
Scan → ICP match against map → Apply correction → Update map → Repeat
```

This is **greedy** — each correction is immediately committed. Two problems:

### Problem 1: Bad ICP = Permanent Damage

```
Step 1: ICP works     Step 2: ICP fails     Step 3: ICP works (but off)
┌──────────┐          ┌──────────┐          ┌──────────┐
│ ■ ■ ■ ■  │          │ ■ ■ ■ ■  │          │ ■ ■ ■ ■  │
│ ■      ■ │   ──►    │ ■      ■ │   ──►    │ ■  ■   ■ │  ← ghost wall!
│ ■  @   ■ │          │ ■    @?  │          │ ■   @  ■ │
│ ■ ■ ■ ■  │          │ ■ ■ ■ ■  │          │ ■ ■ ■ ■  │
└──────────┘          └──────────┘          └──────────┘
     OK                  Drift!               Ghosting!
```

When ICP fails at step 2, the pose drifts. Step 3's scan lands in the wrong place → ghost wall. And it's permanent — there's no undo.

### Problem 2: No Loop Closure

```
Robot explores a loop:

Start → ─ ─ ─ ─ ─ ─ ─ ─ ─ ┐
                             │
                             ↓
  ┌─ ─ ─ ─ ─ ─ ─ ─ ─ End   │
  │                          │
  └ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─┘

End should be at Start, but accumulated drift means
the robot thinks it's 10cm away from where it began.
```

A greedy system has no way to say "I'm back at Start" and fix the whole trajectory.

### The Solution: Defer and Optimize

Instead of immediately committing each ICP correction, **record it as a constraint** and periodically find the set of poses that best satisfies ALL constraints at once.

```
Greedy:    scan → correct → commit (no going back!)

Graph:     scan → record constraint → ... → optimize all at once → rebuild map
```

---

## Part 2: The Pose Graph Structure

### Nodes and Edges

```
    Odometry         Odometry         Odometry
 N0 ═══════════► N1 ═══════════► N2 ═══════════► N3
  ║               ║               ║               ║
  ║   ICP         ║   ICP         ║   ICP         ║
  ╠───(edge)──────╣───(edge)──────╣───(edge)──────╝
  ║                                               ║
  ║              Loop closure                     ║
  ╚═══════════════════════════════════════════════╝
```

**Node** = A pose where the robot stopped and scanned.

- Stores: `(x, y, θ)` + the raw scan data

**Edge** = A constraint saying "the relative pose between these two nodes should be approximately THIS."

- Stores: measured relative transform `(dx, dy, dθ)` + how confident we are

**Three types of edges:**

| Edge           | Source           | Connects          | Confidence                     |
| -------------- | ---------------- | ----------------- | ------------------------------ |
| Odometry       | Encoders + IMU   | Consecutive nodes | Medium (wheel slip, IMU drift) |
| ICP sequential | Scan-to-scan ICP | Consecutive nodes | Variable (match quality)       |
| Loop closure   | Scan-to-scan ICP | Distant nodes     | High (if ICP succeeds)         |

### Why TWO Edges Between Consecutive Nodes?

Odometry and ICP measure the same motion differently. Having both gives the optimizer more information:

- If they agree → high confidence in the transform
- If they disagree → optimizer down-weights the less consistent one

---

## Part 3: Information Matrices (Confidence)

Each edge has an **information matrix** Ω (3×3) that says how much to trust it.

### Intuition: Springs

Think of edges as **springs** connecting nodes:

```
     Stiff spring (high Ω)         Loose spring (low Ω)
     ═══════════════════            ~~~~~~~~~~~~~~~~~~
     "I'm VERY sure about          "I'm not sure about
      this transform"                this transform"
```

When you optimize, you're finding the node positions that minimize the total spring energy. Stiff springs (high confidence edges) dominate.

This is why a bad ICP match doesn't ruin everything: it gets a loose spring, and the optimizer mostly ignores it.

### Building the Information Matrix

For a diagonal information matrix (sufficient for this project):

```
     ┌ 1/σ²_x    0       0    ┐
Ω =  │   0     1/σ²_y     0    │
     └   0       0     1/σ²_θ  ┘
```

Where σ is the standard deviation of the measurement uncertainty.

**Odometry:** σ grows with distance traveled.

```python
σ_x = σ_y = 0.05 * distance_cm   # 5% of distance
σ_θ = 0.02 * abs(rotation_deg)   # 2% of rotation
```

**ICP:** σ depends on match quality.

```python
# Good match (80% ratio, 2cm error) → tight spring
σ_x = σ_y ≈ 0.5 cm
σ_θ ≈ 1°

# Bad match (30% ratio, 8cm error) → loose spring
σ_x = σ_y ≈ 5 cm
σ_θ ≈ 10°
```

---

## Part 4: Optimization with scipy

### What the Optimizer Does

Given:

- Current poses: `[N0, N1, N2, ...]`
- Edges with measurements and information matrices

**Find** adjusted poses that minimize total weighted error:

```
E = Σ (predicted_transform - measured_transform)ᵀ Ω (predicted_transform - measured_transform)
    edge
```

This is a **nonlinear least squares** problem — exactly what `scipy.optimize.least_squares` solves.

### The Residual Function

You provide one function to scipy: `residual(poses_flat)` → returns a vector of errors.

For each edge between node i and node j:

```python
# 1. Extract poses from flat vector
xi, yi, θi = poses_flat[3*i : 3*i+3]
xj, yj, θj = poses_flat[3*j : 3*j+3]

# 2. Predicted relative transform (in node i's local frame)
dx_pred =  cos(θi) * (xj - xi) + sin(θi) * (yj - yi)
dy_pred = -sin(θi) * (xj - xi) + cos(θi) * (yj - yi)
dθ_pred = θj - θi

# 3. Error = predicted - measured
error = [dx_pred - dx_meas, dy_pred - dy_meas, normalize(dθ_pred - dθ_meas)]

# 4. Weight by sqrt of information matrix
weighted_error = sqrt(Ω) @ error
```

Stack all weighted errors into one big vector → return it.

### The normalize() Function

**Critical.** Without it, the optimizer thinks 1° and 359° are 358° apart:

```python
def normalize_angle(θ):
    return (θ + π) % (2π) - π
```

### The Anchor

If every pose can move, the whole graph can slide around while keeping edges happy. Fix node 0 to prevent this:

- Either exclude node 0's variables from the optimization vector
- Or add a very strong "prior" edge pinning it to its initial position

### Huber Loss

`scipy.optimize.least_squares(residual, x0, loss='huber')` automatically down-weights large residuals. This means:

- Normal edges → squared error (standard least squares)
- Outlier edges (bad ICP) → linear error (much less influence)

No special outlier code needed — the loss function handles it.

---

## Part 5: Loop Closure

### Detection

When the robot is near a previously visited position (but many scans ago), that's a loop closure opportunity.

```python
def find_loop_candidates(current_pose, min_gap=10, max_dist=50):
    candidates = []
    for node in graph.nodes:
        if current_node_id - node.id < min_gap:
            continue  # Too recent, skip
        if distance(current_pose, node.pose) < max_dist:
            candidates.append(node.id)
    return candidates
```

### Verification

Run scan-to-scan ICP between the current scan and the candidate's scan. If it converges with good quality → add a loop closure edge.

### The Magic

When a loop closure edge is added and we optimize, the correction **propagates backward** through the entire chain:

```
Before optimization:                After optimization:
N0 ─── N1 ─── N2 ─── N3            N0 ─── N1 ─── N2 ─── N3
                      │                     ↑      ↑      ↑
                loop closure                └──────┴──────┘
                      │             All poses shift slightly to
                      ▼             satisfy the loop closure
                     N0
```

Every node between N0 and N3 shifts a little bit to absorb the error, rather than putting all the correction on one pose.

---

## Part 6: Map Rebuild

After optimization adjusts poses, the occupancy grid is stale (built with old poses). Solution:

```python
def rebuild_map(grid, graph):
    grid.reset()  # Clear to all-unknown
    for node in graph.nodes.values():
        grid.update(node.scan_points, node.pose, free_rays=node.free_rays)
```

This is fast (Bresenham ray tracing) and guarantees a clean map with no ghosting.

When to rebuild: after every optimization call (every 5 scans).

---

## Part 7: Connecting to Your Code

### What Changes

| Component            | Before (greedy)                     | After (graph)                               |
| -------------------- | ----------------------------------- | ------------------------------------------- |
| `_scan_and_update()` | ICP → apply correction → update map | ICP → add edge → (maybe optimize + rebuild) |
| Pose tracking        | Modified in-place by ICP            | Stored in graph nodes                       |
| Map                  | Updated incrementally               | Rebuilt from graph after optimization       |
| ICP target           | Scan vs accumulated map             | Scan vs previous scan                       |

### What Stays the Same

- Scanner, hardware, motion control
- Frontier detection and path planning
- Web UI (just add graph visualization data)
- A\* and occupancy grid internals

### New File Structure

```
src/
├── pose_graph.py          # [NEW] PoseNode, PoseEdge, PoseGraph + optimizer
├── graph_slam_system.py   # [NEW] GraphSlamSystem extends SlamSystem
├── slam_system.py         # [UNCHANGED] Original greedy SLAM (fallback)
├── icp.py                 # [UNCHANGED] Used for scan-to-scan matching
├── occupancy_grid.py      # [UNCHANGED] Rebuilt after optimization
└── ...
```

---

## Part 8: Implementation Order

1. **`PoseGraph` basics** — `add_node()`, `add_edge()`, `get_node()`, properties
2. **Information matrices** — `compute_odometry_info_matrix()`, `compute_icp_info_matrix()`
3. **`_normalize_angle()`** — one line but critical
4. **`_residual()`** — the core optimization function
5. **`optimize()`** — wire up scipy
6. **`rebuild_map()`** — clear + replay
7. **`GraphSlamSystem._scan_and_update()`** — integration
8. **`find_loop_candidates()`** + `_check_loop_closures()` — loop closure
9. **Test with synthetic data** before running on the robot

### Testing Strategy

Start with `pose_graph_test.py`:

```python
# Test 1: Add nodes and edges, verify they're stored correctly
# Test 2: Build a square loop with known transforms, optimize,
#          verify poses form a perfect square
# Test 3: Add one bad edge, verify Huber loss down-weights it
# Test 4: Loop closure — verify backward propagation
# Test 5: rebuild_map — verify grid matches expected
```

---

## Summary

| Concept                | What It Means for Your Robot                             |
| ---------------------- | -------------------------------------------------------- |
| **Node**               | "I was HERE and saw THIS"                                |
| **Edge**               | "Between HERE and THERE, I measured THIS transform"      |
| **Information matrix** | "I'm THIS confident about that measurement"              |
| **Optimize**           | "Find the poses that make ALL edges happy at once"       |
| **Loop closure**       | "I'm back where I was 20 scans ago — fix the whole path" |
| **Rebuild**            | "Replay all scans at corrected poses for a clean map"    |
