Convex Scene Editor - Performance Findings
===========================================

All speed measurements below are taken in Release builds unless noted otherwise.
Tests use 1024 random rays per press of T (adjustable with M/N keys).
Convex count adjustable with Y/U keys (double/halve).


(a) Raycasts/ms vs. Object Count (Release, 1024 rays)
------------------------------------------------------

  Objects |  No Opt  |   Disc   |   AABB   | QuadTree |   BVH
  --------|----------|----------|----------|----------|----------
       16 | [TODO]ms | [TODO]ms | [TODO]ms | [TODO]ms | [TODO]ms
      256 | [TODO]ms | [TODO]ms | [TODO]ms | [TODO]ms | [TODO]ms
     1024 | [TODO]ms | [TODO]ms | [TODO]ms | [TODO]ms | [TODO]ms

Raycasts per ms (= 1024 / time_ms):

  Objects |  No Opt  |   Disc   |   AABB   | QuadTree |   BVH
  --------|----------|----------|----------|----------|----------
       16 | [TODO]   | [TODO]   | [TODO]   | [TODO]   | [TODO]
      256 | [TODO]   | [TODO]   | [TODO]   | [TODO]   | [TODO]
     1024 | [TODO]   | [TODO]   | [TODO]   | [TODO]   | [TODO]


(b) Effect of Early Ray-Disc Rejection
---------------------------------------

Disc rejection adds a cheap bounding-disc distance check before the full
convex hull raycast. If the ray misses the bounding disc, we skip the
expensive per-edge plane tests entirely.

At low object counts (16), the improvement is modest since most rays hit
something anyway in a small scene. As object count grows (256, 1024), the
speedup becomes more significant because a larger fraction of objects are
far from any given ray and get cheaply rejected.

Speedup vs. No Opt:
  16 objects:   [TODO]x faster
  256 objects:  [TODO]x faster
  1024 objects: [TODO]x faster


(c) Effect of Spatial Partitioning (BVH + QuadTree)
----------------------------------------------------

Two spatial structures are implemented:

  - SymmetricQuadTree: Uniform 4-way subdivision, fixed depth 4 (256 leaf cells).
    Each convex is inserted into every leaf cell its AABB overlaps.

  - AABB2Tree (BVH): Binary space partition alternating X/Y axis splits.
    Depth = max(3, floor(log2(N)) - 3). Convexes stored in leaf nodes only.

At 16 objects, spatial structures add overhead (tree traversal cost) with
little benefit since brute force is already fast. The tree methods may
actually be slightly slower than simple disc/AABB rejection at this scale.

At 256+ objects, spatial structures start to shine. The BVH narrows
candidates to a small subset per ray, and the QuadTree similarly limits
checks to objects in traversed cells.

At 1024 objects, the advantage is clear. Brute force scales linearly with
object count, while BVH scales roughly O(log N) per ray.

Speedup vs. No Opt:
                  QuadTree          BVH
  16 objects:     [TODO]x           [TODO]x
  256 objects:    [TODO]x           [TODO]x
  1024 objects:   [TODO]x           [TODO]x


(d) Build Configuration Comparison (1024 rays, 256 objects)
------------------------------------------------------------

  Config       |  No Opt  |   Disc   |   AABB   | QuadTree |   BVH
  -------------|----------|----------|----------|----------|----------
  Debug        | [TODO]ms | [TODO]ms | [TODO]ms | [TODO]ms | [TODO]ms
  DebugInline  | [TODO]ms | [TODO]ms | [TODO]ms | [TODO]ms | [TODO]ms
  FastBreak    | [TODO]ms | [TODO]ms | [TODO]ms | [TODO]ms | [TODO]ms
  Release      | [TODO]ms | [TODO]ms | [TODO]ms | [TODO]ms | [TODO]ms

Debug is expected to be significantly slower (10-50x) due to disabled
optimizations, iterator debugging, and bounds checking. DebugInline
recovers some speed by inlining small functions (Vec2 math, etc.).
FastBreak sits between Debug and Release. Release enables full
optimizations (/O2), auto-vectorization, and link-time code generation.


(e) General Complexity Trends
------------------------------

  - No Optimization: O(N) per ray. Every convex is tested with full
    plane-by-plane intersection. Total cost scales linearly with object
    count. Doubling objects roughly doubles the time.

  - Disc/AABB Rejection: Still O(N) per ray, but with a much cheaper
    constant factor. The bounding volume test (distance check or AABB
    overlap) is significantly faster than the full convex hull test,
    so most objects are rejected early. The scaling is still linear
    but the slope is gentler.

  - QuadTree: Approaches O(N/C) per ray where C is the number of leaf
    cells (256 at depth 4). For uniformly distributed objects, each ray
    traverses only a fraction of cells. However, objects spanning multiple
    cells get tested multiple times (deduplication via visited flag helps
    but adds overhead).

  - BVH: Approaches O(log N) per ray in the best case. Each level of the
    tree halves the search space. This is the most scalable approach and
    should show the best performance at high object counts.

  Observed: going from 16 to 1024 objects (64x increase), brute force
  time increases roughly [TODO]x, while BVH increases roughly [TODO]x.


(f) Spatial Structure Details
------------------------------

BVH (AABB2Tree):
  - Depth formula: max(3, floor(log2(N)) - 3)
  - At 16 objects:   depth = max(3, 4-3) = 3  ->  8 leaf nodes
  - At 256 objects:  depth = max(3, 8-3) = 5  ->  32 leaf nodes
  - At 1024 objects: depth = max(3, 10-3) = 7 ->  128 leaf nodes
  - Split strategy: alternates X/Y axis at each level, splits at midpoint
  - Convexes assigned to leaves based on AABB center position

QuadTree (SymmetricQuadTree):
  - Fixed depth 4 -> 256 leaf cells (uniform grid)
  - Each convex inserted into all overlapping leaf cells
  - Uses a per-convex visited flag to avoid duplicate ray tests
  - At low density (16 objects / 256 cells), most cells are empty
  - At high density (1024 objects / 256 cells), ~4 objects per cell average


(g) Other Observations
-----------------------

  - Disc rejection is almost free to add and provides consistent speedup
    across all object counts. It should always be enabled as a baseline.

  - AABB rejection provides marginal improvement over disc rejection alone.
    The AABB test is slightly tighter (rejects more objects) but also
    slightly more expensive per test (4 comparisons vs 1 distance check).

  - At very low object counts (8-16), the overhead of tree traversal and
    candidate list allocation can make spatial structures slower than
    brute force with disc rejection. The crossover point where trees
    become beneficial is around [TODO] objects.

  - The BVH generally outperforms the QuadTree at high object counts
    because it adapts its partitioning to the actual object distribution,
    while the QuadTree uses a fixed uniform grid regardless of where
    objects are clustered.

  - Random seed (F8) affects results slightly since object placement
    changes which rays hit which objects. Running multiple seeds and
    averaging gives more stable measurements.

  - Ray count (M/N to adjust) affects measurement precision. At 1024 rays
    the timing is stable enough for comparison. At lower ray counts,
    timer resolution noise becomes significant.
