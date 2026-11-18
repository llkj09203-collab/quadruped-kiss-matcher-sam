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
