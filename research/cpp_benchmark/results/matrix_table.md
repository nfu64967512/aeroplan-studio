# Coverage-path-planning efficiency benchmark

Common settings for every method: alt 140 m, HFOV 60 deg => swath 161.7 m, strip pitch 113.2 m (30% side overlap), reference speed 20 m/s, every track starts and ends at home (0,0) (audited: max |first-home| = max |last-home| = 0.0 m for every method). Every number below comes from `bench.metrics`; no planner reports its own length or coverage.

Reproduce: `JOBS=10 PYTHONUTF8=1 python -u run_matrix.py` (one subprocess per cell, 228 cells) then `python -u make_table.py`. Two independent full runs of the matrix agreed on **all 228 cells to the last printed digit** for both `total_km` and `scan_rate`, so nothing here is run-to-run noise.

**Provenance warning.** The two project planners were measured against the *working tree*, not a commit: git HEAD `c565319` plus uncommitted edits, with `mission/swarm_coordinator.py` md5 `0c9cb713...` and `core/dccpp/dccpp_path_builder.py` md5 `82463450...`. Another session was editing those same files while this ran, so re-running later can legitimately give different DCCPP/HARVEST numbers. The 15 baselines are self-contained and import nothing from the project.

| id | area | n |
|:--|:--|:--|
| S1 | single 5.0x4.0 km (20.0 km2) | 2, 4, 8, 16 |
| S2 | 4 blocks 1.25x4.0 km, separated (20.0 km2) | 4, 8 |
| S3 | thin 8.0x1.0 km (8.0 km2) | 4, 8 |
| S4 | L concave, 4x4 minus 2x2 (12.0 km2) | 4, 8 |
| S5 | large 10x8 km (80.0 km2) | 8, 16 |

`+` = coverage < 0.99 in that cell, so its efficiency numbers are meaningless: the cell is excluded from ranks and winners. `*empty*` = the planner returned no flyable path at all. **bold** = winner of that column.

### Table 0 - headline comparison, pure geometry (the paper table)

Zero-radius polylines: this is the ruler the brief defines.

`eff index` = mean over all 12 cells of (this method's scan rate / the best scan rate in that cell). A cell where the method returns an empty plan scores 0 -- not covering is not rewarded with a missing value. `eff|ok` is the same mean restricted to the cells where it did produce a plan, so the two columns separate *how good it is* from *how often it works*. `mean ovh` averages route overhead over the cells that pass the coverage gate; the floor at pitch = 0.7 x swath is **1.429**, not 1.0.

| rank | method | family | eff index | eff-when-ok | mean ovh | turn charge % | cell wins | cov<0.99 | empty plan | cells w/ idle aircraft |
|--:|:--|:--|--:|--:|--:|--:|--:|--:|--:|--:|
| 1 | `spiral_v2` | spiral | 0.986 | 0.986 | 1.602 | 7.6% | 6 | 0 | 0 | 0 |
| 2 | `spiral_v2_r1` | spiral | 0.977 | 0.977 | 1.590 | 6.4% | 0 | 0 | 0 | 0 |
| 3 | `spiral` | spiral | 0.963 | 0.963 | 1.614 | 6.6% | 0 | 0 | 0 | 0 |
| 4 | `grid_nn_v2` | grid+NN | 0.957 | 0.957 | 1.656 | 46.2% | 2 | 0 | 0 | 0 |
| 5 | `grid_nn_v2b` | grid+NN | 0.953 | 0.953 | 1.661 | 47.5% | 4 | 0 | 0 | 0 |
| 6 | `grid_nn` | grid+NN | 0.925 | 0.925 | 1.702 | 37.6% | 0 | 0 | 0 | 0 |
| 7 | `darp_v2_r1` | DARP | 0.902 | 0.902 | 1.708 | 8.8% | 0 | 0 | 0 | 0 |
| 8 | `darp_v2` | DARP | 0.902 | 0.902 | 1.718 | 10.9% | 0 | 0 | 0 | 0 |
| 9 | `bous_v2_rev1` | boustrophedon | 0.900 | 0.900 | 1.722 | 6.8% | 0 | 0 | 0 | 0 |
| 10 | `bous_v2` | boustrophedon | 0.887 | 0.887 | 1.772 | 5.4% | 0 | 0 | 0 | 0 |
| 11 | `darp` | DARP | 0.886 | 0.886 | 1.711 | 8.5% | 0 | 0 | 0 | 0 |
| 12 | `bous_contiguous` | boustrophedon | 0.882 | 0.882 | 1.778 | 5.3% | 0 | 0 | 0 | 0 |
| 13 | `bous_axis_aligned` | boustrophedon | 0.875 | 0.875 | 1.792 | 6.3% | 0 | 0 | 0 | 0 |
| 14 | `boustrophedon` | boustrophedon | 0.761 | 0.761 | 1.812 | 7.9% | 0 | 0 | 0 | 6 |
| 15 | `DCCPP_mix` | **this project** | 0.713 | 0.713 | 1.973 | 8.7% | 0 | 0 | 0 | 0 |
| 16 | `DCCPP` | **this project** | 0.713 | 0.713 | 1.973 | 8.7% | 0 | 0 | 0 | 0 |
| 17 | `bous_interleaved` | boustrophedon | 0.689 | 0.689 | 2.096 | 7.1% | 0 | 0 | 0 | 0 |
| 18 | `HARVEST` | **this project** | 0.477 | 0.573 | 2.420 | 0.2% | 0 | 2 | 2 | 0 |
| 19 | `HARVEST_mix` | **this project** | 0.225 | 0.449 | 2.082 | 0.2% | 0 | 0 | 6 | 3 |

### Table 0b - headline comparison, charged for a 200 m turn radius

Same ruler, but every corner pays the arc it would actually have to fly. Read this one if the deliverable is a fixed-wing/VTOL mission rather than a geometry result.

`eff index` = mean over all 12 cells of (this method's scan rate / the best scan rate in that cell). A cell where the method returns an empty plan scores 0 -- not covering is not rewarded with a missing value. `eff|ok` is the same mean restricted to the cells where it did produce a plan, so the two columns separate *how good it is* from *how often it works*. `mean ovh` averages route overhead over the cells that pass the coverage gate; the floor at pitch = 0.7 x swath is **1.429**, not 1.0.

| rank | method | family | eff index | eff-when-ok | mean ovh | turn charge % | cell wins | cov<0.99 | empty plan | cells w/ idle aircraft |
|--:|:--|:--|--:|--:|--:|--:|--:|--:|--:|--:|
| 1 | `spiral_v2_r1` | spiral | 0.998 | 0.998 | 1.590 | 6.4% | 7 | 0 | 0 | 0 |
| 2 | `spiral` | spiral | 0.977 | 0.977 | 1.614 | 6.6% | 3 | 0 | 0 | 0 |
| 3 | `spiral_v2` | spiral | 0.947 | 0.947 | 1.602 | 7.6% | 1 | 0 | 0 | 0 |
| 4 | `bous_v2` | boustrophedon | 0.929 | 0.929 | 1.772 | 5.4% | 3 | 0 | 0 | 0 |
| 5 | `bous_contiguous` | boustrophedon | 0.929 | 0.929 | 1.778 | 5.3% | 3 | 0 | 0 | 0 |
| 6 | `bous_v2_rev1` | boustrophedon | 0.921 | 0.921 | 1.722 | 6.8% | 2 | 0 | 0 | 0 |
| 7 | `bous_axis_aligned` | boustrophedon | 0.915 | 0.915 | 1.792 | 6.3% | 4 | 0 | 0 | 0 |
| 8 | `darp` | DARP | 0.894 | 0.894 | 1.711 | 8.5% | 0 | 0 | 0 | 0 |
| 9 | `darp_v2_r1` | DARP | 0.892 | 0.892 | 1.708 | 8.8% | 0 | 0 | 0 | 0 |
| 10 | `darp_v2` | DARP | 0.873 | 0.873 | 1.718 | 10.9% | 0 | 0 | 0 | 0 |
| 11 | `boustrophedon` | boustrophedon | 0.775 | 0.775 | 1.812 | 7.9% | 0 | 0 | 0 | 6 |
| 12 | `DCCPP_mix` | **this project** | 0.740 | 0.740 | 1.973 | 8.7% | 0 | 0 | 0 | 0 |
| 13 | `DCCPP` | **this project** | 0.740 | 0.740 | 1.973 | 8.7% | 0 | 0 | 0 | 0 |
| 14 | `bous_interleaved` | boustrophedon | 0.729 | 0.729 | 2.096 | 7.1% | 0 | 0 | 0 | 0 |
| 15 | `grid_nn` | grid+NN | 0.693 | 0.693 | 1.702 | 37.6% | 0 | 0 | 0 | 0 |
| 16 | `grid_nn_v2b` | grid+NN | 0.688 | 0.688 | 1.661 | 47.5% | 0 | 0 | 0 | 0 |
| 17 | `grid_nn_v2` | grid+NN | 0.676 | 0.676 | 1.656 | 46.2% | 0 | 0 | 0 | 0 |
| 18 | `HARVEST` | **this project** | 0.536 | 0.643 | 2.420 | 0.2% | 0 | 2 | 2 | 0 |
| 19 | `HARVEST_mix` | **this project** | 0.252 | 0.503 | 2.082 | 0.2% | 0 | 0 | 6 | 3 |

### Table 1 - scan rate (km2/h, higher is better)

scan_rate = covered area / (longest track / 20 m/s). This is the makespan metric: it is set by the slowest aircraft, so it punishes both long routes and bad load balance.

| method | S1/2 | S1/4 | S1/8 | S1/16 | S2/4 | S2/8 | S3/4 | S3/8 | S4/4 | S4/8 | S5/8 | S5/16 | mean rank | valid cells |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| `spiral_v2` | 15.82 | 30.60 | 57.34 | **99.54** | **29.85** | **52.04** | **32.11** | **61.08** | 29.64 | 56.05 | 60.90 | **112.44** | 2.17 | 12/12 |
| `grid_nn_v2` | 16.16 | 31.85 | **58.94** | 98.09 | 28.61 | 51.84 | 28.55 | 44.11 | 30.18 | **56.85** | 60.50 | 110.39 | 3.08 | 12/12 |
| `grid_nn_v2b` | **16.33** | **32.42** | 58.89 | 98.66 | 28.92 | 51.76 | 28.02 | 40.84 | **30.65** | 54.67 | **61.03** | 110.69 | 3.08 | 12/12 |
| `spiral_v2_r1` | 15.81 | 30.65 | 56.95 | 97.83 | 28.81 | 51.02 | 31.98 | 60.45 | 29.38 | 55.78 | 61.02 | 111.20 | 3.25 | 12/12 |
| `spiral` | 15.81 | 30.42 | 56.69 | 97.83 | 27.00 | 46.76 | 31.98 | 60.45 | 29.38 | 55.78 | 60.18 | 111.20 | 5.42 | 12/12 |
| `grid_nn` | 15.62 | 31.06 | 58.17 | 93.74 | 26.66 | 48.36 | 27.89 | 42.03 | 29.17 | 54.11 | 60.90 | 109.48 | 6.75 | 12/12 |
| `bous_v2_rev1` | 15.46 | 30.07 | 54.43 | 94.90 | 27.13 | 48.65 | 23.87 | 42.06 | 28.38 | 50.44 | 59.73 | 109.84 | 7.25 | 12/12 |
| `bous_v2` | 15.46 | 30.07 | 54.43 | 91.16 | 27.13 | 48.65 | 23.79 | 35.76 | 28.23 | 50.44 | 59.73 | 108.87 | 8.75 | 12/12 |
| `darp_v2` | 15.07 | 29.70 | 56.41 | 95.26 | 26.84 | 46.99 | 25.74 | 45.23 | 27.71 | 49.73 | 59.05 | 109.69 | 8.92 | 12/12 |
| `darp_v2_r1` | 15.03 | 29.84 | 56.32 | 97.15 | 26.84 | 46.99 | 25.49 | 45.93 | 27.88 | 49.02 | 59.05 | 108.02 | 9.25 | 12/12 |
| `bous_contiguous` | 15.46 | 30.07 | 54.43 | 91.16 | 27.13 | 48.65 | 23.79 | 35.76 | 27.33 | 49.17 | 59.73 | 108.87 | 10.00 | 12/12 |
| `darp` | 15.00 | 29.36 | 55.34 | 92.95 | 26.84 | 46.44 | 25.08 | 43.58 | 26.90 | 48.57 | 59.05 | 107.21 | 11.42 | 12/12 |
| `bous_axis_aligned` | 15.46 | 30.07 | 54.43 | 91.16 | 25.73 | 46.59 | 23.79 | 35.76 | 27.14 | 49.17 | 59.73 | 108.87 | 11.83 | 12/12 |
| `boustrophedon` | 14.70 | 27.70 | 45.50 | 66.95 | 26.84 | 41.34 | 17.86 | 23.85 | 25.97 | 40.42 | 55.85 | 91.55 | 14.00 | 12/12 |
| `DCCPP_mix` | 13.86 | 26.34 | 44.63 | 67.03 | 24.95 | 41.22 | 17.80 | 23.85 | 19.28 | 32.66 | 54.16 | 90.19 | 15.17 | 12/12 |
| `DCCPP` | 13.86 | 26.34 | 44.63 | 67.03 | 24.95 | 41.22 | 17.80 | 23.85 | 19.28 | 32.66 | 54.16 | 90.19 | 16.17 | 12/12 |
| `bous_interleaved` | 14.38 | 26.29 | 41.40 | 58.10 | 23.02 | 33.96 | 17.52 | 23.15 | 22.93 | 33.79 | 52.61 | 82.90 | 16.67 | 12/12 |
| `HARVEST` | *empty* | 22.33+ | 31.19+ | 45.94 | 22.79 | 37.20 | 16.14 | 20.42 | 17.60 | 31.95 | *empty* | 66.83 | 17.88 | 8/12 |
| `HARVEST_mix` | *empty* | *empty* | *empty* | *empty* | 21.64 | 35.58 | 10.66 | 10.66 | 15.56 | 15.56 | *empty* | *empty* | 18.83 | 6/12 |

### Table 2 - route overhead (total length / ideal, lower is better)

overhead = total flown / (covered area / swath). The floor at 30% overlap is 1.429; everything above that is turns, transit to and from home, and double-scanned ground.

| method | S1/2 | S1/4 | S1/8 | S1/16 | S2/4 | S2/8 | S3/4 | S3/8 | S4/4 | S4/8 | S5/8 | S5/16 | mean rank | valid cells |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| `spiral_v2_r1` | 1.472 | 1.509 | 1.607 | 1.844 | 1.592 | 1.797 | 1.436 | 1.480 | 1.571 | 1.618 | **1.517** | 1.636 | 3.42 | 12/12 |
| `grid_nn_v2b` | **1.425** | **1.435** | **1.559** | 1.822 | 1.607 | **1.786** | 1.658 | 2.228 | **1.517** | 1.690 | 1.523 | 1.679 | 3.83 | 12/12 |
| `spiral` | 1.472 | 1.528 | 1.622 | 1.844 | 1.693 | 1.947 | **1.436** | **1.480** | 1.571 | **1.618** | 1.525 | 1.636 | 4.42 | 12/12 |
| `spiral_v2` | 1.472 | 1.521 | 1.624 | 1.871 | **1.557** | 1.787 | 1.450 | 1.524 | 1.571 | 1.661 | 1.529 | 1.656 | 4.42 | 12/12 |
| `grid_nn_v2` | 1.440 | 1.459 | 1.575 | 1.885 | 1.613 | 1.792 | 1.628 | 2.098 | 1.537 | 1.629 | 1.538 | 1.675 | 5.00 | 12/12 |
| `grid_nn` | 1.489 | 1.498 | 1.589 | 1.898 | 1.722 | 1.911 | 1.663 | 2.157 | 1.587 | 1.705 | 1.518 | 1.691 | 7.67 | 12/12 |
| `bous_v2_rev1` | 1.501 | 1.535 | 1.667 | 1.844 | 1.705 | 1.901 | 1.938 | 1.975 | 1.629 | 1.799 | 1.536 | **1.632** | 8.50 | 12/12 |
| `bous_contiguous` | 1.501 | 1.535 | 1.667 | 1.812 | 1.705 | 1.901 | 1.954 | 2.594 | 1.684 | 1.805 | 1.536 | 1.641 | 8.83 | 12/12 |
| `bous_v2` | 1.501 | 1.535 | 1.667 | 1.812 | 1.705 | 1.901 | 1.954 | 2.594 | 1.638 | 1.783 | 1.536 | 1.641 | 9.17 | 12/12 |
| `bous_axis_aligned` | 1.501 | 1.535 | 1.667 | **1.812** | 1.799 | 1.968 | 1.954 | 2.594 | 1.674 | 1.818 | 1.536 | 1.641 | 9.25 | 12/12 |
| `darp` | 1.549 | 1.559 | 1.636 | 1.897 | 1.675 | 1.861 | 1.715 | 1.905 | 1.724 | 1.806 | 1.549 | 1.651 | 9.42 | 12/12 |
| `darp_v2_r1` | 1.549 | 1.559 | 1.635 | 1.880 | 1.675 | 1.837 | 1.707 | 1.891 | 1.668 | 1.857 | 1.549 | 1.686 | 9.58 | 12/12 |
| `darp_v2` | 1.544 | 1.567 | 1.635 | 1.897 | 1.675 | 1.834 | 1.805 | 1.899 | 1.672 | 1.861 | 1.549 | 1.684 | 9.75 | 12/12 |
| `boustrophedon` | 1.542 | 1.628 | 1.757 | 1.977 | 1.675 | 1.959 | 1.955 | 2.275 | 1.712 | 1.857 | 1.627 | 1.784 | 12.67 | 12/12 |
| `DCCPP` | 1.636 | 1.715 | 1.831 | 2.139 | 1.806 | 1.951 | 2.117 | 2.754 | 2.081 | 2.185 | 1.659 | 1.806 | 15.25 | 12/12 |
| `HARVEST_mix` | - | - | - | - | 2.030 | 2.239 | 1.985 | 1.985 | 2.125 | 2.125 | - | - | 15.33 | 6/12 |
| `DCCPP_mix` | 1.636 | 1.715 | 1.831 | 2.139 | 1.806 | 1.951 | 2.117 | 2.754 | 2.081 | 2.185 | 1.659 | 1.806 | 16.25 | 12/12 |
| `bous_interleaved` | 1.576 | 1.725 | 1.991 | 2.406 | 2.014 | 2.544 | 2.146 | 2.768 | 1.910 | 2.308 | 1.747 | 2.015 | 17.17 | 12/12 |
| `HARVEST` | - | 2.080+ | 2.303+ | 2.742 | 1.994 | 2.269 | 2.436 | 3.053 | 2.162 | 2.487 | - | 2.218 | 18.38 | 8/12 |

### Table 2b - scan rate after charging a 200 m turn radius (km2/h, higher is better)

Tables 0-2 are pure geometry, which silently subsidises any method that plans corners a real aircraft cannot fly. Here every vertex is charged the arc-vs-corner difference for a 200 m minimum turn radius (the class-A airframe in `bench._drones`); a >170 deg reversal is charged a pi*R teardrop. Coverage is unchanged. This is a *lower bound* on the penalty, not a re-plan: a Dubins-aware planner would have chosen a different route rather than pay it.

| method | S1/2 | S1/4 | S1/8 | S1/16 | S2/4 | S2/8 | S3/4 | S3/8 | S4/4 | S4/8 | S5/8 | S5/16 | mean rank | valid cells |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| `spiral_v2_r1` | 14.73 | 28.48 | **53.67** | 84.75 | **27.59** | **47.53** | 27.43 | **49.04** | **27.39** | **48.67** | **59.21** | 105.12 | 2.08 | 12/12 |
| `bous_v2` | 14.79 | 28.75 | 51.83 | 83.47 | 25.36 | 44.52 | 22.53 | 33.29 | 25.70 | 45.44 | 58.39 | **105.16** | 4.33 | 12/12 |
| `bous_v2_rev1` | **14.79** | **28.75** | 51.83 | 83.93 | 25.36 | 44.52 | 20.93 | 34.88 | 26.14 | 45.44 | 58.39 | 95.60 | 4.67 | 12/12 |
| `spiral` | 14.73 | 27.91 | 50.17 | 84.75 | 25.50 | 44.17 | 27.43 | 49.04 | 27.39 | 48.67 | 57.91 | 105.12 | 4.67 | 12/12 |
| `bous_contiguous` | 14.79 | 28.75 | 51.83 | 83.47 | 25.36 | 44.52 | 22.53 | 33.29 | 25.23 | 45.97 | 58.39 | 105.16 | 5.00 | 12/12 |
| `spiral_v2` | 14.42 | 26.89 | 47.85 | 76.47 | 26.88 | 45.66 | **27.63** | 43.24 | 27.17 | 43.34 | 57.42 | 105.03 | 5.50 | 12/12 |
| `bous_axis_aligned` | 14.79 | 28.75 | 51.83 | **85.30** | 22.79 | 40.66 | 22.53 | 33.29 | 24.96 | 45.97 | 58.39 | 105.16 | 5.83 | 12/12 |
| `darp_v2_r1` | 14.14 | 27.66 | 50.16 | 78.27 | 25.77 | 45.12 | 21.41 | 30.33 | 24.37 | 41.75 | 55.82 | 101.15 | 7.33 | 12/12 |
| `darp` | 14.21 | 27.34 | 50.16 | 73.48 | 25.77 | 43.31 | 22.08 | 38.87 | 23.27 | 40.78 | 55.84 | 100.69 | 8.08 | 12/12 |
| `darp_v2` | 14.38 | 27.00 | 49.83 | 73.48 | 25.77 | 44.68 | 22.13 | 29.83 | 24.03 | 38.04 | 55.84 | 93.30 | 8.08 | 12/12 |
| `boustrophedon` | 14.09 | 25.95 | 41.63 | 57.73 | 25.76 | 38.40 | 17.01 | 22.52 | 21.10 | 33.01 | 53.44 | 86.67 | 11.67 | 12/12 |
| `DCCPP_mix` | 13.29 | 24.42 | 38.52 | 60.25 | 24.01 | 39.70 | 16.96 | 22.52 | 17.88 | 28.61 | 51.60 | 85.61 | 13.33 | 12/12 |
| `DCCPP` | 13.29 | 24.42 | 38.52 | 60.25 | 24.01 | 39.70 | 16.96 | 22.52 | 17.88 | 28.61 | 51.60 | 85.61 | 14.33 | 12/12 |
| `bous_interleaved` | 13.79 | 24.99 | 38.98 | 54.13 | 21.82 | 32.29 | 16.70 | 21.89 | 21.33 | 31.79 | 51.27 | 80.37 | 14.58 | 12/12 |
| `grid_nn` | 11.26 | 20.68 | 37.36 | 62.02 | 18.97 | 32.87 | 18.86 | 27.63 | 20.28 | 32.60 | 40.96 | 71.63 | 14.58 | 12/12 |
| `grid_nn_v2b` | 11.67 | 21.76 | 37.06 | 57.52 | 19.58 | 35.53 | 18.25 | 26.01 | 19.01 | 32.62 | 39.81 | 69.71 | 14.92 | 12/12 |
| `grid_nn_v2` | 11.09 | 20.98 | 34.29 | 59.92 | 18.88 | 33.02 | 19.12 | 24.81 | 20.38 | 31.53 | 39.01 | 69.21 | 15.58 | 12/12 |
| `HARVEST` | - | 22.24+ | 31.16+ | 45.63 | 22.76 | 37.16 | 16.13 | 20.40 | 17.58 | 31.91 | - | 66.79 | 16.50 | 8/12 |
| `HARVEST_mix` | - | - | - | - | 21.61 | 35.54 | 10.65 | 10.65 | 15.54 | 15.54 | - | - | 17.83 | 6/12 |

### Table 3 - per-cell winner vs this project's two planners

| cell | winner | scan | ovh | best DCCPP variant | best HARVEST variant | project's best rank |
|:--|:--|--:|--:|--:|--:|--:|
| S1/n=2 | `grid_nn_v2b` | 16.33 | 1.425 | 13.86 (-15.1%, `DCCPP_mix`) | *empty plan* (`HARVEST_mix`) | 16 of 17 |
| S1/n=4 | `grid_nn_v2b` | 32.42 | 1.435 | 26.34 (-18.8%, `DCCPP_mix`) | 22.33+ (-31.1%, `HARVEST`) | 15 of 17 |
| S1/n=8 | `grid_nn_v2` | 58.94 | 1.575 | 44.63 (-24.3%, `DCCPP_mix`) | 31.19+ (-47.1%, `HARVEST`) | 15 of 17 |
| S1/n=16 | `spiral_v2` | 99.54 | 1.871 | 67.03 (-32.7%, `DCCPP_mix`) | 45.94 (-53.8%, `HARVEST`) | 14 of 18 |
| S2/n=4 | `spiral_v2` | 29.85 | 1.557 | 24.95 (-16.4%, `DCCPP_mix`) | 22.79 (-23.7%, `HARVEST`) | 15 of 19 |
| S2/n=8 | `spiral_v2` | 52.04 | 1.787 | 41.22 (-20.8%, `DCCPP_mix`) | 37.20 (-28.5%, `HARVEST`) | 15 of 19 |
| S3/n=4 | `spiral_v2` | 32.11 | 1.450 | 17.80 (-44.6%, `DCCPP_mix`) | 16.14 (-49.7%, `HARVEST`) | 15 of 19 |
| S3/n=8 | `spiral_v2` | 61.08 | 1.524 | 23.85 (-61.0%, `DCCPP_mix`) | 20.42 (-66.6%, `HARVEST`) | 15 of 19 |
| S4/n=4 | `grid_nn_v2b` | 30.65 | 1.517 | 19.28 (-37.1%, `DCCPP_mix`) | 17.60 (-42.6%, `HARVEST`) | 16 of 19 |
| S4/n=8 | `grid_nn_v2` | 56.85 | 1.629 | 32.66 (-42.6%, `DCCPP_mix`) | 31.95 (-43.8%, `HARVEST`) | 16 of 19 |
| S5/n=8 | `grid_nn_v2b` | 61.03 | 1.523 | 54.16 (-11.3%, `DCCPP_mix`) | *empty plan* (`HARVEST_mix`) | 15 of 17 |
| S5/n=16 | `spiral_v2` | 112.44 | 1.656 | 90.19 (-19.8%, `DCCPP_mix`) | 66.83 (-40.6%, `HARVEST`) | 15 of 18 |

### Table 5 - head-to-head: does either project planner ever win?

Cells where both methods produced a valid plan (coverage >= 0.99, non-empty). `w/c` = cells won / cells comparable; margin is the median of (project scan rate - baseline scan rate) / baseline.

| baseline | DCCPP w/c (geom) | median | DCCPP w/c (R200) | median | HARVEST w/c (geom) | median | HARVEST w/c (R200) | median |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|
| `boustrophedon` | 1/12 | -1.9% | 2/12 | -3.4% | 0/8 | -15.1% | 0/8 | -9.4% |
| `bous_contiguous` | 0/12 | -17.2% | 0/12 | -18.6% | 0/8 | -35.0% | 0/8 | -30.3% |
| `bous_axis_aligned` | 0/12 | -17.2% | 1/12 | -18.6% | 0/8 | -35.0% | 0/8 | -29.6% |
| `bous_interleaved` | 9/12 | +3.0% | 7/12 | +1.5% | 1/8 | -7.8% | 3/8 | -3.4% |
| `bous_v2` | 0/12 | -17.2% | 0/12 | -18.6% | 0/8 | -36.7% | 0/8 | -29.8% |
| `bous_v2_rev1` | 0/12 | -17.9% | 0/12 | -15.1% | 0/8 | -36.7% | 0/8 | -29.8% |
| `spiral` | 0/12 | -18.9% | 0/12 | -18.6% | 0/8 | -40.1% | 0/8 | -35.8% |
| `spiral_v2` | 0/12 | -20.8% | 0/12 | -18.5% | 0/8 | -40.6% | 0/8 | -35.3% |
| `spiral_v2_r1` | 0/12 | -19.2% | 0/12 | -18.6% | 0/8 | -40.1% | 0/8 | -35.8% |
| `grid_nn` | 0/12 | -17.6% | 7/12 | +18.0% | 0/8 | -39.7% | 2/8 | -6.7% |
| `grid_nn_v2` | 0/12 | -20.5% | 8/12 | +16.4% | 0/8 | -41.7% | 3/8 | -3.5% |
| `grid_nn_v2b` | 0/12 | -20.4% | 8/12 | +11.7% | 0/8 | -41.6% | 2/8 | -4.2% |
| `darp` | 0/12 | -15.9% | 0/12 | -15.0% | 0/8 | -34.6% | 0/8 | -24.4% |
| `darp_v2` | 0/12 | -17.8% | 0/12 | -11.1% | 0/8 | -36.5% | 0/8 | -26.8% |
| `darp_v2_r1` | 0/12 | -16.5% | 0/12 | -15.4% | 0/8 | -36.7% | 0/8 | -24.7% |

### Table 4 - failures (empty plan, coverage < 0.99, or idle aircraft)

| method | cell | aircraft used / asked | coverage | what happened |
|:--|:--|--:|--:|:--|
| `boustrophedon` | S1/n=8 | 7 / 8 | 0.9962 | 1 aircraft got no work |
| `boustrophedon` | S1/n=16 | 12 / 16 | 0.9962 | 4 aircraft got no work |
| `boustrophedon` | S3/n=4 | 3 / 4 | 1.0000 | 1 aircraft got no work |
| `boustrophedon` | S3/n=8 | 5 / 8 | 1.0000 | 3 aircraft got no work |
| `boustrophedon` | S4/n=8 | 7 / 8 | 0.9930 | 1 aircraft got no work |
| `boustrophedon` | S5/n=16 | 15 / 16 | 1.0000 | 1 aircraft got no work |
| `HARVEST` | S1/n=2 | 0 / 2 | 0.0000 | planner returned no flyable path |
| `HARVEST` | S1/n=4 | 4 / 4 | 0.9796 | coverage gate failed -- efficiency numbers void |
| `HARVEST` | S1/n=8 | 8 / 8 | 0.9825 | coverage gate failed -- efficiency numbers void |
| `HARVEST` | S5/n=8 | 0 / 8 | 0.0000 | planner returned no flyable path |
| `HARVEST_mix` | S1/n=2 | 0 / 2 | 0.0000 | planner returned no flyable path |
| `HARVEST_mix` | S1/n=4 | 0 / 4 | 0.0000 | planner returned no flyable path |
| `HARVEST_mix` | S1/n=8 | 0 / 8 | 0.0000 | planner returned no flyable path |
| `HARVEST_mix` | S1/n=16 | 0 / 16 | 0.0000 | planner returned no flyable path |
| `HARVEST_mix` | S3/n=4 | 2 / 4 | 1.0000 | 2 aircraft got no work |
| `HARVEST_mix` | S3/n=8 | 2 / 8 | 1.0000 | 6 aircraft got no work |
| `HARVEST_mix` | S4/n=8 | 4 / 8 | 1.0000 | 4 aircraft got no work |
| `HARVEST_mix` | S5/n=8 | 0 / 8 | 0.0000 | planner returned no flyable path |
| `HARVEST_mix` | S5/n=16 | 0 / 16 | 0.0000 | planner returned no flyable path |

### Table 6 - where DCCPP's makespan gap comes from

scan rate is (covered area) / (longest track), and longest track = (total / n) x balance. So the gap against the cell winner factorises into a **route-length** term and a **load-balance** term. `waste` is the share of swept ground that lies outside the work area -- ferry legs that scan nothing useful.

| cell | DCCPP total km | winner total km | length gap | DCCPP balance | winner balance | DCCPP waste | winner waste | winner |
|:--|--:|--:|--:|--:|--:|--:|--:|:--|
| S1/n=2 | 202.3 | 176.3 | +14.8% | 1.027 | 1.001 | 3.7% | 1.2% | `grid_nn_v2b` |
| S1/n=4 | 212.1 | 177.6 | +19.5% | 1.031 | 1.001 | 3.6% | 1.2% | `grid_nn_v2b` |
| S1/n=8 | 226.4 | 194.8 | +16.2% | 1.140 | 1.003 | 3.6% | 1.8% | `grid_nn_v2` |
| S1/n=16 | 264.6 | 231.4 | +14.3% | 1.299 | 1.000 | 3.5% | 0.0% | `spiral_v2` |
| S2/n=4 | 223.5 | 192.5 | +16.1% | 1.033 | 1.002 | 15.2% | 7.7% | `spiral_v2` |
| S2/n=8 | 241.4 | 220.9 | +9.3% | 1.158 | 1.001 | 17.9% | 9.5% | `spiral_v2` |
| S3/n=4 | 104.8 | 71.7 | +46.1% | 1.236 | 1.000 | 8.2% | -0.0% | `spiral_v2` |
| S3/n=8 | 136.3 | 75.4 | +80.8% | 1.418 | 1.000 | 8.2% | 0.0% | `spiral_v2` |
| S4/n=4 | 154.4 | 112.6 | +37.2% | 1.160 | 1.001 | 19.2% | 4.0% | `grid_nn_v2b` |
| S4/n=8 | 162.1 | 120.9 | +34.1% | 1.305 | 1.005 | 19.1% | 2.9% | `grid_nn_v2` |
| S5/n=8 | 818.2 | 753.5 | +8.6% | 1.036 | 1.002 | 1.9% | 1.4% | `grid_nn_v2b` |
| S5/n=16 | 890.4 | 819.6 | +8.6% | 1.144 | 1.000 | 1.8% | 0.0% | `spiral_v2` |

