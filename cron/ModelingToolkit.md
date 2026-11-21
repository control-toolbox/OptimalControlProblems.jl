# Cron report: ModelingToolkit

<sub>`CronModelingToolkit.yml` · daily (04:00 UTC), a rotation with a budget of 1 h (#435) · Julia 1.13, Ubuntu + Windows, CPU</sub>

Generated: 2026-11-21T04:48:35+00:00
Run: https://github.com/control-toolbox/OptimalControlProblems.jl/actions/runs/44864074081
Commit: `99bccaa81d34bb093092805a70d5faebc107bafb`
Julia: `1.13`
Package version: `0.5.0`

| platform | test | problem | modelings | status | outcome |
| --- | --- | --- | --- | --- | --- |
| cron-ModelingToolkit-1.13-ubuntu-latest | test_convergence shooting=single solver=ipopt | beam | ModelingToolkit | valid | pass |
| cron-ModelingToolkit-1.13-windows-latest | test_convergence shooting=single solver=ipopt | beam | ModelingToolkit | valid | pass |
| cron-ModelingToolkit-1.13-ubuntu-latest | test_convergence shooting=single solver=ipopt | bioreactor | ModelingToolkit | valid | pass |
| cron-ModelingToolkit-1.13-windows-latest | test_convergence shooting=single solver=ipopt | bioreactor | ModelingToolkit | valid | pass |
| cron-ModelingToolkit-1.13-ubuntu-latest | test_convergence shooting=single solver=ipopt | bryson_denham | ModelingToolkit | valid | pass |
| cron-ModelingToolkit-1.13-windows-latest | test_convergence shooting=single solver=ipopt | bryson_denham | ModelingToolkit | valid | pass |
| cron-ModelingToolkit-1.13-ubuntu-latest | test_convergence shooting=single solver=ipopt | chain | ModelingToolkit | valid | pass |
| cron-ModelingToolkit-1.13-windows-latest | test_convergence shooting=single solver=ipopt | chain | ModelingToolkit | valid | pass |
| cron-ModelingToolkit-1.13-ubuntu-latest | test_convergence shooting=single solver=ipopt | double_oscillator | ModelingToolkit | valid | pass |
| cron-ModelingToolkit-1.13-windows-latest | test_convergence shooting=single solver=ipopt | double_oscillator | ModelingToolkit | valid | pass |
| cron-ModelingToolkit-1.13-ubuntu-latest | test_convergence shooting=single solver=ipopt | electric_vehicle | ModelingToolkit | valid | pass |
| cron-ModelingToolkit-1.13-windows-latest | test_convergence shooting=single solver=ipopt | electric_vehicle | ModelingToolkit | valid | pass |
| cron-ModelingToolkit-1.13-ubuntu-latest | test_convergence shooting=single solver=ipopt | jackson | ModelingToolkit | valid | pass |
| cron-ModelingToolkit-1.13-windows-latest | test_convergence shooting=single solver=ipopt | jackson | ModelingToolkit | valid | pass |
| cron-ModelingToolkit-1.13-ubuntu-latest | test_convergence shooting=single solver=ipopt | mountain_car | ModelingToolkit | valid | pass |
| cron-ModelingToolkit-1.13-windows-latest | test_convergence shooting=single solver=ipopt | mountain_car | ModelingToolkit | valid | pass |
| cron-ModelingToolkit-1.13-ubuntu-latest | test_convergence shooting=single solver=ipopt | robbins | ModelingToolkit | valid | pass |
| cron-ModelingToolkit-1.13-windows-latest | test_convergence shooting=single solver=ipopt | robbins | ModelingToolkit | valid | pass |
| cron-ModelingToolkit-1.13-ubuntu-latest | test_convergence shooting=single solver=ipopt | vanderpol | ModelingToolkit | valid | pass |
| cron-ModelingToolkit-1.13-windows-latest | test_convergence shooting=single solver=ipopt | vanderpol | ModelingToolkit | valid | pass |

## Convergence (reported, not tested)

| platform | problem | modeling | configuration | result | reason | objective | iterations | time | gap | solution | change |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| cron-ModelingToolkit-1.13-ubuntu-latest | beam | ModelingToolkit | shooting=single solver=ipopt | converged | Success | 8.89866 | 11 | 28.1979 | 0.00109876 |  | new |
| cron-ModelingToolkit-1.13-windows-latest | beam | ModelingToolkit | shooting=single solver=ipopt | converged | Success | 8.89866 | 11 | 28.3376 | 0.00109876 |  | gained |
| cron-ModelingToolkit-1.13-ubuntu-latest | bioreactor | ModelingToolkit | shooting=single solver=ipopt | converged | Success | -12.9174 | 438 | 45.8633 |  |  | new |
| cron-ModelingToolkit-1.13-windows-latest | bioreactor | ModelingToolkit | shooting=single solver=ipopt | converged | Success | -12.9174 | 307 | 44.976 |  |  | new |
| cron-ModelingToolkit-1.13-ubuntu-latest | bryson_denham | ModelingToolkit | shooting=single solver=ipopt | converged | Success | 4.00365 | 13 | 24.7289 | 0.00091242 |  | new |
| cron-ModelingToolkit-1.13-windows-latest | bryson_denham | ModelingToolkit | shooting=single solver=ipopt | converged | Success | 4.00365 | 13 | 24.7729 | 0.00091242 |  | new |
| cron-ModelingToolkit-1.13-ubuntu-latest | chain | ModelingToolkit | shooting=single solver=ipopt | converged | Success | 5.06859 | 327 | 23.8405 |  |  | new |
| cron-ModelingToolkit-1.13-windows-latest | chain | ModelingToolkit | shooting=single solver=ipopt | converged | Success | 5.06859 | 213 | 23.8459 |  |  | new |
| cron-ModelingToolkit-1.13-ubuntu-latest | double_oscillator | ModelingToolkit | shooting=single solver=ipopt | converged | Success | 0.000911035 | 16 | 10.3682 |  |  | new |
| cron-ModelingToolkit-1.13-windows-latest | double_oscillator | ModelingToolkit | shooting=single solver=ipopt | converged | Success | 0.000911035 | 16 | 11.7622 |  |  | new |
| cron-ModelingToolkit-1.13-ubuntu-latest | electric_vehicle | ModelingToolkit | shooting=single solver=ipopt | converged | Success | 1229.08 | 9 | 21.0468 |  |  | new |
| cron-ModelingToolkit-1.13-windows-latest | electric_vehicle | ModelingToolkit | shooting=single solver=ipopt | converged | Success | 1229.08 | 9 | 22.4969 |  |  | new |
| cron-ModelingToolkit-1.13-ubuntu-latest | jackson | ModelingToolkit | shooting=single solver=ipopt | converged | Success | -0.191789 | 82 | 43.2087 |  |  | new |
| cron-ModelingToolkit-1.13-windows-latest | jackson | ModelingToolkit | shooting=single solver=ipopt | converged | Success | -0.191789 | 82 | 46.1046 |  |  | new |
| cron-ModelingToolkit-1.13-ubuntu-latest | mountain_car | ModelingToolkit | shooting=single solver=ipopt | converged | Success | 102.64 | 32 | 32.215 |  |  | new |
| cron-ModelingToolkit-1.13-windows-latest | mountain_car | ModelingToolkit | shooting=single solver=ipopt | converged | Success | 102.64 | 32 | 34.757 |  |  | new |
| cron-ModelingToolkit-1.13-ubuntu-latest | robbins | ModelingToolkit | shooting=single solver=ipopt | converged | Success | 20.3006 | 16 | 25.2399 |  |  | new |
| cron-ModelingToolkit-1.13-windows-latest | robbins | ModelingToolkit | shooting=single solver=ipopt | converged | Success | 20.3006 | 16 | 25.1383 |  |  | new |
| cron-ModelingToolkit-1.13-ubuntu-latest | vanderpol | ModelingToolkit | shooting=single solver=ipopt | converged | Success | 1.0479 | 11 | 10.6602 |  |  | new |
| cron-ModelingToolkit-1.13-windows-latest | vanderpol | ModelingToolkit | shooting=single solver=ipopt | converged | Success | 1.0479 | 11 | 10.9328 |  |  | new |

## Rotation (#435)

Each run tests the problems tested longest ago, while its time budget lasts; the tables above show the last result of every problem.

| problem | last tested | run |
| --- | --- | --- |
| beam | 2026-11-21 04:48 (this run) | [44864074081](https://github.com/control-toolbox/OptimalControlProblems.jl/actions/runs/44864074081) |
| bioreactor | 2026-11-21 04:48 (this run) | [44864074081](https://github.com/control-toolbox/OptimalControlProblems.jl/actions/runs/44864074081) |
| bryson_denham | 2026-11-21 04:48 (this run) | [44864074081](https://github.com/control-toolbox/OptimalControlProblems.jl/actions/runs/44864074081) |
| chain | 2026-11-21 04:48 (this run) | [44864074081](https://github.com/control-toolbox/OptimalControlProblems.jl/actions/runs/44864074081) |
| double_oscillator | 2026-11-21 04:48 (this run) | [44864074081](https://github.com/control-toolbox/OptimalControlProblems.jl/actions/runs/44864074081) |
| electric_vehicle | 2026-11-21 04:48 (this run) | [44864074081](https://github.com/control-toolbox/OptimalControlProblems.jl/actions/runs/44864074081) |
| jackson | 2026-11-21 04:48 (this run) | [44864074081](https://github.com/control-toolbox/OptimalControlProblems.jl/actions/runs/44864074081) |
| mountain_car | 2026-11-21 04:48 (this run) | [44864074081](https://github.com/control-toolbox/OptimalControlProblems.jl/actions/runs/44864074081) |
| robbins | 2026-11-21 04:48 (this run) | [44864074081](https://github.com/control-toolbox/OptimalControlProblems.jl/actions/runs/44864074081) |
| vanderpol | 2026-11-21 04:48 (this run) | [44864074081](https://github.com/control-toolbox/OptimalControlProblems.jl/actions/runs/44864074081) |
