# ICRA experiment readiness

The GCOPTER branch is ready for controlled baseline development, but not yet for frozen paper statistics.

## Implemented

- FIRI, PCA/Frenet OBB-SFC, and Liu/DecompUtil EllipsoidDecomp all feed their H-polytopes into the same `GCOPTER_PolytopeSFC` setup and optimizer.
- Strict runs disable FIRI fallback and log generation/setup/optimization failures.
- Schema v9 records timing, geometry, trajectory results, constrained piece count, sampled corridor penalty/violation before and after optimization, plus map/route seeds, fixed endpoints and key map/dynamics parameters needed to audit fair comparisons.
- `tf_firi` is a paired native-FIRI variant with a route-directional MVIE term and hard face budget. It is the first core-algorithm stage, not yet the final MINCO-sensitivity method.

## Still required

1. Pin the ROS, Eigen, OMPL and DecompUtil revisions and verify clean Release builds.
2. Use the same route-search seed, map, start/goal, dynamics, time limit and failure denominator across FIRI, OBB-SFC and EllipsoidDecomp.
3. Add continuous or conservative trajectory-versus-corridor verification; current violation metrics are quadrature-sampled diagnostics.
4. Add an execution- or validator-level goal-arrival/collision result. The current `success` means that optimization returned a non-empty trajectory, not that a vehicle completed a mission.
5. Run at least 30 independent trials per scenario and report confidence intervals and tail latency.
6. Complete ablations for corridor construction, overlap threshold/extension, face count and segment length.
7. Before claiming full TF-SFC, replace the route-direction proxy in `tf_firi` with an initialized-MINCO sensitivity/transport direction and validate a bounded second corridor pass.
8. Keep DecompROS/RViz visualization evidence separate from algorithmic and optimizer evidence.

The cross-planner protocol and EGO-specific grouping rules are documented in the EGO branch's readiness file.
