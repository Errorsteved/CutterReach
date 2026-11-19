# Cutter accessibility after cutterhead indexing

Each cutter ID counts once per design. The table gives cutterhead stop angles in degrees about the +Z axis from mechanical zero. The cutters and the supports on the back of the cutterhead rotate together.

Design selection tested 24 stop angles and the refined evaluation tested 72. The holdout set included 9 cutters evaluated at 72 stop angles each.

- The **Baseline** arm reached 3/10 cutters at one or more stops and 3/10 at mechanical zero alone.
- The **Optimized** arm reached 5/10 cutters at one or more stops and 5/10 at mechanical zero alone.

| Cutter ID | Radius (m) | Baseline feasible / tested stops | Baseline selected stop | Optimized feasible / tested stops | Optimized selected stop |
|---|---:|---:|---:|---:|---:|
| 01 | 0.500 | 45/72 | 0° | 45/72 | 0° |
| 02 | 0.850 | 45/72 | 0° | 45/72 | 0° |
| 03 | 1.200 | 0/72 | — | 0/72 | — |
| 04 | 1.550 | 0/72 | — | 0/72 | — |
| 05 | 1.900 | 0/72 | — | 0/72 | — |
| 06 | 2.250 | 0/72 | — | 0/72 | — |
| 07 | 2.600 | 45/72 | 0° | 45/72 | 0° |
| 08 | 2.950 | 0/72 | — | 45/72 | 0° |
| 09 | 3.300 | 0/72 | — | 45/72 | 0° |
| 10 | 3.650 | 0/72 | — | 0/72 | — |

Each successful solution verifies a static service pose. A cutter with no solution on the sampled grid may still be reachable at another stop or joint configuration. Disassembly, extraction, retrieval and reinstallation paths remain unverified. The result files list joint configurations, pose errors and clearances for each cutter, along with outcomes and collision causes at each stop. [Per-cutter results](cutters.csv) · [Per-stop results](targets.csv)

## Design comparison and validation

MATLAB 26.2.0.3386108 (R2026b). RNG seed 30386. The baseline and selected design share 30000 normalized workspace samples.

The search admitted 16 of 20 structural candidates. The selected upper arm measures 0.900 m and the telescope operates between 1.420 and 2.295 m.

| Evaluation | Design | Cutters with bounded poses | Accessible cutters / total | Access | Mean η, all | Mean η, reached | Mean η, paired |
|---|---|---:|---:|---:|---:|---:|---:|
| Selection | Baseline | 7 | 3 / 10 | 30.0% | 0.0640 | 0.2132 | 0.2132 |
| Selection | Optimized | 9 | 5 / 10 | 50.0% | 0.1012 | 0.2024 | 0.2167 |
| Refined | Baseline | 7 | 3 / 10 | 30.0% | 0.0640 | 0.2132 | 0.2132 |
| Refined | Optimized | 9 | 5 / 10 | 50.0% | 0.1013 | 0.2026 | 0.2168 |
| Holdout | Baseline | 7 | 4 / 9 | 44.4% | 0.0953 | 0.2145 | 0.2145 |
| Holdout | Optimized | 8 | 5 / 9 | 55.6% | 0.1175 | 0.2116 | 0.2191 |

The design advantage was +20.0 percentage points on the selection grid and +20.0 on the refined grid. The held-out cutters yielded +11.1 points.

Among the 3 refined cutters reached by both arms, the paired mean dexterity changed from 0.2132 to 0.2168. The all-target score assigns zero to cutters without a witness.

Uniform joint sampling produced 3413 (11.38%) clear baseline configurations and 2975 (9.92%) clear selected configurations. This fraction weights joint configurations. Cutter access weights cutter IDs equally. Workspace sampling uses the reference head angle.

- baselineCutters — 3 without a bounded pose, 4 with bounded poses all blocked, 3 with a free witness.
- optimizedCutters — 1 without a bounded pose, 4 with bounded poses all blocked, 5 with a free witness.

## Published baseline checks

The Cartesian residuals for paper points A and B are 2.903e-13 and 1.299 mm. A uses an extension below the Table 1 minimum. B falls within the published bounds. All 7 Table 2 joint rows yielded a bounded IK witness for the frame reconstructed from that row.

- X range in metres — paper [-2.9, 3.6], sampled [-2.759, 3.812].
- Y range in metres — paper [-3.3, 3.3], sampled [-3.295, 3.266].
- Z range in metres — paper [-0.2, 4.6], sampled [-2.904, 6.039].

Two explicit configurations inside Table 1 give Z = -2.070 m and Z = 6.262 m. Their joint vectors are in paper_axial_examples.csv.

The largest axis discrepancy is retained in the results. The publication gives no original Fig. 10 samples or executable limit/filter settings.

The registered cutter stations, link radii, support layout and packaging rules are course assumptions. A success certifies the final static pose in this geometry. A search miss is resolution-dependent.
