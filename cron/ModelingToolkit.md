# Cron report: ModelingToolkit

<sub>`CronModelingToolkit.yml` · daily (04:00 UTC), a rotation with a budget of 1 h (#435) · Julia 1.13, Ubuntu + Windows, CPU</sub>

Generated: 2026-11-28T04:40:47+00:00
Run: https://github.com/control-toolbox/OptimalControlProblems.jl/actions/runs/46105625778
Commit: `e9e0004c7c11f27f2776bf3a798ecb2c73822752`
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
| cron-ModelingToolkit-1.13-ubuntu-latest | beam | ModelingToolkit | shooting=single solver=ipopt | converged | Success | 8.89866 | 11 | 26.9124 | 0.00109876 |  |  |
| cron-ModelingToolkit-1.13-windows-latest | beam | ModelingToolkit | shooting=single solver=ipopt | converged | Success | 8.89866 | 11 | 27.9277 | 0.00109876 |  |  |
| cron-ModelingToolkit-1.13-ubuntu-latest | bioreactor | ModelingToolkit | shooting=single solver=ipopt | converged | Success | -12.9174 | 438 | 46.4451 |  |  |  |
| cron-ModelingToolkit-1.13-windows-latest | bioreactor | ModelingToolkit | shooting=single solver=ipopt | converged | Success | -12.9174 | 307 | 42.7083 |  |  |  |
| cron-ModelingToolkit-1.13-ubuntu-latest | bryson_denham | ModelingToolkit | shooting=single solver=ipopt | converged | Success | 4.00365 | 13 | 24.1817 | 0.00091242 |  |  |
| cron-ModelingToolkit-1.13-windows-latest | bryson_denham | ModelingToolkit | shooting=single solver=ipopt | converged | Success | 4.00365 | 13 | 24.4127 | 0.00091242 |  |  |
| cron-ModelingToolkit-1.13-ubuntu-latest | chain | ModelingToolkit | shooting=single solver=ipopt | converged | Success | 5.06859 | 327 | 23.8713 |  |  |  |
| cron-ModelingToolkit-1.13-windows-latest | chain | ModelingToolkit | shooting=single solver=ipopt | converged | Success | 5.06859 | 213 | 22.7397 |  |  |  |
| cron-ModelingToolkit-1.13-ubuntu-latest | double_oscillator | ModelingToolkit | shooting=single solver=ipopt | converged | Success | 0.000911035 | 16 | 10.9868 |  |  |  |
| cron-ModelingToolkit-1.13-windows-latest | double_oscillator | ModelingToolkit | shooting=single solver=ipopt | converged | Success | 0.000911035 | 16 | 10.895 |  |  |  |
| cron-ModelingToolkit-1.13-ubuntu-latest | electric_vehicle | ModelingToolkit | shooting=single solver=ipopt | converged | Success | 1229.08 | 9 | 21.9759 |  |  |  |
| cron-ModelingToolkit-1.13-windows-latest | electric_vehicle | ModelingToolkit | shooting=single solver=ipopt | converged | Success | 1229.08 | 9 | 21.5408 |  |  |  |
| cron-ModelingToolkit-1.13-ubuntu-latest | jackson | ModelingToolkit | shooting=single solver=ipopt | converged | Success | -0.191789 | 82 | 43.5925 |  |  |  |
| cron-ModelingToolkit-1.13-windows-latest | jackson | ModelingToolkit | shooting=single solver=ipopt | converged | Success | -0.191789 | 82 | 45.0211 |  |  |  |
| cron-ModelingToolkit-1.13-ubuntu-latest | mountain_car | ModelingToolkit | shooting=single solver=ipopt | converged | Success | 102.64 | 32 | 33.6984 |  |  |  |
| cron-ModelingToolkit-1.13-windows-latest | mountain_car | ModelingToolkit | shooting=single solver=ipopt | converged | Success | 102.64 | 32 | 33.1589 |  |  |  |
| cron-ModelingToolkit-1.13-ubuntu-latest | robbins | ModelingToolkit | shooting=single solver=ipopt | converged | Success | 20.3006 | 16 | 25.9065 |  |  |  |
| cron-ModelingToolkit-1.13-windows-latest | robbins | ModelingToolkit | shooting=single solver=ipopt | converged | Success | 20.3006 | 16 | 25.3616 |  |  |  |
| cron-ModelingToolkit-1.13-ubuntu-latest | vanderpol | ModelingToolkit | shooting=single solver=ipopt | converged | Success | 1.0479 | 11 | 10.5957 |  |  |  |
| cron-ModelingToolkit-1.13-windows-latest | vanderpol | ModelingToolkit | shooting=single solver=ipopt | converged | Success | 1.0479 | 11 | 10.5231 |  |  |  |

## Rotation (#435)

Each run tests the problems tested longest ago, while its time budget lasts; the tables above show the last result of every problem.

| problem | last tested | run |
| --- | --- | --- |
| beam | 2026-11-28 04:40 (this run) | [46105625778](https://github.com/control-toolbox/OptimalControlProblems.jl/actions/runs/46105625778) |
| bioreactor | 2026-11-28 04:40 (this run) | [46105625778](https://github.com/control-toolbox/OptimalControlProblems.jl/actions/runs/46105625778) |
| bryson_denham | 2026-11-28 04:40 (this run) | [46105625778](https://github.com/control-toolbox/OptimalControlProblems.jl/actions/runs/46105625778) |
| chain | 2026-11-28 04:40 (this run) | [46105625778](https://github.com/control-toolbox/OptimalControlProblems.jl/actions/runs/46105625778) |
| double_oscillator | 2026-11-28 04:40 (this run) | [46105625778](https://github.com/control-toolbox/OptimalControlProblems.jl/actions/runs/46105625778) |
| electric_vehicle | 2026-11-28 04:40 (this run) | [46105625778](https://github.com/control-toolbox/OptimalControlProblems.jl/actions/runs/46105625778) |
| jackson | 2026-11-28 04:40 (this run) | [46105625778](https://github.com/control-toolbox/OptimalControlProblems.jl/actions/runs/46105625778) |
| mountain_car | 2026-11-28 04:40 (this run) | [46105625778](https://github.com/control-toolbox/OptimalControlProblems.jl/actions/runs/46105625778) |
| robbins | 2026-11-28 04:40 (this run) | [46105625778](https://github.com/control-toolbox/OptimalControlProblems.jl/actions/runs/46105625778) |
| vanderpol | 2026-11-28 04:40 (this run) | [46105625778](https://github.com/control-toolbox/OptimalControlProblems.jl/actions/runs/46105625778) |
