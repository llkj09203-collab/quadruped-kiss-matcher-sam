# Scan Context standardization baseline

This baseline freezes the detector behavior before adding Sector Key alignment
or Ring Key KD-tree retrieval. It complements the mapping regression baseline:
performance changes are accepted only when candidate behavior remains explainable.

## Reference algorithm

- Original paper: <https://gisbi-kim.github.io/publications/gkim-2018-iros.pdf>
- Official implementation: <https://github.com/SignalImageCV/scancontext>

The reference search has two hierarchical stages: Ring Key nearest-neighbor
candidate retrieval, followed by Sector Key coarse yaw alignment and full Scan
Context distance evaluation in a local shift window. The baseline implementation
uses linear Ring Key retrieval and evaluates all 60 shifts for each shortlisted
candidate.

## Dataset and configuration

- Mapping product: `refactor_final_20261118`
- Keyframes: 205
- Rings / sectors: 20 / 60
- Maximum radius: 80 m
- Excluded recent keyframes: 30
- Candidate count: 10
- Minimum time separation: 30 s
- Acceptance distance: 0.18
- CPU used for the recorded timing: Intel Core Ultra 5 125H

Run from the ROS 2 workspace root:

```bash
./build/kiss_matcher_ros/scan_context_benchmark \
  /tmp/kiss_sam_refactor_final/refactor_final_20261118 \
  /tmp/scan_context_baseline.csv
```

## Frozen behavior

- Queries: 205
- Accepted Scan Context candidates: 174
- Final query: query 204, match 1, distance 0.000070603, yaw 2.722713633 rad
- Deterministic SHA-256 of CSV columns `query_id,match_id,distance,yaw_rad`:
  `8048ccc16ca1d29ab8b92324565a121108d17834069499f945f38b9e440f829a`

The large number of descriptor-level candidates is expected for this small,
repetitive laboratory sequence. KISS-Matcher remains the geometric verifier.
Standardization must not silently turn this descriptor threshold into a final
loop acceptance decision.

## Timing observations

Five Release-build runs produced the following retrieval means:

```text
0.216690 ms
0.219680 ms
0.211821 ms
0.242454 ms
0.221157 ms
```

The representative median-query time is approximately 0.27 ms. These numbers
are a regression reference for this machine, not a portable real-time limit.
The KD-tree benefit must also be evaluated on a synthetic or campus-scale map;
205 keyframes are too few to demonstrate asymptotic improvement.

## Sector Key standardization result

The first standardization step adds the column-mean Sector Key. It searches all
Sector Key circular shifts for a coarse yaw, then evaluates the full descriptor
only inside the reference local window. With 60 sectors and search ratio 0.1,
this reduces each shortlisted candidate from 60 to 7 full shift evaluations.

On the same 205-keyframe product:

- accepted descriptor candidates remain 174;
- total full shift evaluations are 11,487 for 1,641 candidate comparisons;
- mean retrieval time is 0.067 ms in the recorded Release run;
- final query 204 remains match 1, distance 0.000070603, yaw 2.722713633 rad;
- 5 of the 174 accepted queries select a different, similarly scored candidate.

Those five changes are an expected consequence of replacing global exhaustive
yaw search with the reference coarse-to-fine local search in a repetitive lab.
They are recorded rather than hidden. Scan Context still proposes candidates;
KISS-Matcher geometric registration remains responsible for loop acceptance.

## Ring Key KD-tree result

The second step replaces linear Ring Key candidate retrieval with the dynamic
KD-tree already provided by KISS-Matcher's nanoflann dependency. Eligible Ring
Keys are inserted incrementally, so the tree does not need a full rebuild and
does not delay newly eligible candidates.

On the 205-keyframe product, all 174 accepted results are identical to the
Sector Key stage in candidate ID, descriptor distance and yaw. The recorded
mean retrieval time is 0.073 ms, slightly above the 0.067 ms linear-search run.
This small sequence is below the scale at which KD-tree asymptotic behavior is
expected to help; a campus-scale benchmark with thousands of keyframes remains
required. The implementation favors exact candidate semantics over periodic
tree rebuilds that temporarily omit newly eligible history.

## Full pipeline regression

The completed Sector Key and dynamic Ring Key KD-tree implementation was
replayed through FAST-LIO and KISS-SAM using the 214-second laboratory bag. The
validated session `scan_context_dynamic_20261118_r4` produced:

- 204 keyframe PCDs, timestamps and odometry poses;
- 204 optimized poses plus the trajectory header;
- a readable final map containing 14,067 points;
- 40 geometrically accepted and 90 rejected loop proposals;
- no crash, invalid numeric trajectory value or missing map product.

The modular-refactor reference produced 205 keyframes and 14,079 map points.
The one-keyframe and 12-point difference is within the documented asynchronous
ROS timer variation; all product count and validity invariants pass.
