# Cron report: JuMP

Generated: 2026-11-07T15:43:40+00:00
Run: https://github.com/control-toolbox/OptimalControlProblems.jl/actions/runs/42387108959
Commit: `0f6b1ec2b60b46fcc5141592fd08794531e1f519`
Julia: `1.13`
Package version: `0.5.0`

| platform | test | problem | modelings | status | outcome |
| --- | --- | --- | --- | --- | --- |
| cron-JuMP-1.13-ubuntu-latest | test_convergence solver=ipopt | balanced_field | JuMP | valid | pass |
| cron-JuMP-1.13-ubuntu-latest | test_convergence solver=ipopt | beam | JuMP | valid | pass |
| cron-JuMP-1.13-ubuntu-latest | test_convergence solver=ipopt | bioreactor | JuMP | valid | pass |
| cron-JuMP-1.13-ubuntu-latest | test_convergence solver=ipopt | brachistochrone | JuMP | valid | pass |
| cron-JuMP-1.13-ubuntu-latest | test_convergence solver=ipopt | bryson_denham | JuMP | draft | pass |
| cron-JuMP-1.13-ubuntu-latest | test_convergence solver=ipopt | cart_pendulum | JuMP | valid | pass |
| cron-JuMP-1.13-ubuntu-latest | test_convergence solver=ipopt | chain | JuMP | valid | pass |
| cron-JuMP-1.13-ubuntu-latest | test_convergence solver=ipopt | dielectrophoretic_particle | JuMP | valid | pass |
| cron-JuMP-1.13-ubuntu-latest | test_convergence solver=ipopt | double_oscillator | JuMP | valid | pass |
| cron-JuMP-1.13-ubuntu-latest | test_convergence solver=ipopt | ducted_fan | JuMP | valid | pass |
| cron-JuMP-1.13-ubuntu-latest | test_convergence solver=ipopt | electric_vehicle | JuMP | valid | pass |
| cron-JuMP-1.13-ubuntu-latest | test_convergence solver=ipopt | glider | JuMP | valid | pass |
| cron-JuMP-1.13-ubuntu-latest | test_convergence solver=ipopt | insurance | JuMP | valid | pass |
| cron-JuMP-1.13-ubuntu-latest | test_convergence solver=ipopt | jackson | JuMP | valid | pass |
| cron-JuMP-1.13-ubuntu-latest | test_convergence solver=ipopt | moonlander | JuMP | valid | pass |
| cron-JuMP-1.13-ubuntu-latest | test_convergence solver=ipopt | mountain_car | JuMP | valid | pass |
| cron-JuMP-1.13-ubuntu-latest | test_convergence solver=ipopt | robbins | JuMP | valid | pass |
| cron-JuMP-1.13-ubuntu-latest | test_convergence solver=ipopt | robot | JuMP | valid | pass |
| cron-JuMP-1.13-ubuntu-latest | test_convergence solver=ipopt | rocket | JuMP | valid | pass |
| cron-JuMP-1.13-ubuntu-latest | test_convergence solver=ipopt | space_shuttle | JuMP | valid | pass |
| cron-JuMP-1.13-ubuntu-latest | test_convergence solver=ipopt | steering | JuMP | valid | pass |
| cron-JuMP-1.13-ubuntu-latest | test_convergence solver=ipopt | vanderpol | JuMP | valid | pass |
| cron-JuMP-1.13-windows-latest | test_convergence solver=ipopt | balanced_field | JuMP | valid | pass |
| cron-JuMP-1.13-windows-latest | test_convergence solver=ipopt | beam | JuMP | valid | pass |
| cron-JuMP-1.13-windows-latest | test_convergence solver=ipopt | bioreactor | JuMP | valid | pass |
| cron-JuMP-1.13-windows-latest | test_convergence solver=ipopt | brachistochrone | JuMP | valid | pass |
| cron-JuMP-1.13-windows-latest | test_convergence solver=ipopt | bryson_denham | JuMP | draft | pass |
| cron-JuMP-1.13-windows-latest | test_convergence solver=ipopt | cart_pendulum | JuMP | valid | pass |
| cron-JuMP-1.13-windows-latest | test_convergence solver=ipopt | chain | JuMP | valid | pass |
| cron-JuMP-1.13-windows-latest | test_convergence solver=ipopt | dielectrophoretic_particle | JuMP | valid | pass |
| cron-JuMP-1.13-windows-latest | test_convergence solver=ipopt | double_oscillator | JuMP | valid | pass |
| cron-JuMP-1.13-windows-latest | test_convergence solver=ipopt | ducted_fan | JuMP | valid | pass |
| cron-JuMP-1.13-windows-latest | test_convergence solver=ipopt | electric_vehicle | JuMP | valid | pass |
| cron-JuMP-1.13-windows-latest | test_convergence solver=ipopt | glider | JuMP | valid | pass |
| cron-JuMP-1.13-windows-latest | test_convergence solver=ipopt | insurance | JuMP | valid | pass |
| cron-JuMP-1.13-windows-latest | test_convergence solver=ipopt | jackson | JuMP | valid | pass |
| cron-JuMP-1.13-windows-latest | test_convergence solver=ipopt | moonlander | JuMP | valid | pass |
| cron-JuMP-1.13-windows-latest | test_convergence solver=ipopt | mountain_car | JuMP | valid | pass |
| cron-JuMP-1.13-windows-latest | test_convergence solver=ipopt | robbins | JuMP | valid | pass |
| cron-JuMP-1.13-windows-latest | test_convergence solver=ipopt | robot | JuMP | valid | pass |
| cron-JuMP-1.13-windows-latest | test_convergence solver=ipopt | rocket | JuMP | valid | pass |
| cron-JuMP-1.13-windows-latest | test_convergence solver=ipopt | space_shuttle | JuMP | valid | pass |
| cron-JuMP-1.13-windows-latest | test_convergence solver=ipopt | steering | JuMP | valid | pass |
| cron-JuMP-1.13-windows-latest | test_convergence solver=ipopt | vanderpol | JuMP | valid | pass |

## Convergence (reported, not tested)

| platform | problem | modeling | configuration | result | reason | objective | iterations | time | gap | solution | change |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| cron-JuMP-1.13-ubuntu-latest | balanced_field | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 771.011 | 17 | 0.100114 | 1.47452e-16 | best |  |
| cron-JuMP-1.13-ubuntu-latest | beam | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 8.88928 | 18 | 0.0209809 | 2.13633e-8 | best |  |
| cron-JuMP-1.13-ubuntu-latest | bioreactor | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -12.9255 | 184 | 1.64259 | 0.000327787 | local |  |
| cron-JuMP-1.13-ubuntu-latest | brachistochrone | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 1.80293 | 18 | 0.241537 | 0.0 | best |  |
| cron-JuMP-1.13-ubuntu-latest | bryson_denham | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 4.00014 | 16 | 0.0185332 | 1.2409e-7 | best |  |
| cron-JuMP-1.13-ubuntu-latest | cart_pendulum | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 1.74397 | 316 | 3.21788 | 2.15388e-9 | best |  |
| cron-JuMP-1.13-ubuntu-latest | chain | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 5.06858 | 14 | 0.0725389 | 5.17461e-13 | best |  |
| cron-JuMP-1.13-ubuntu-latest | dielectrophoretic_particle | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 7.81132 | 13 | 0.0397383 | 0.0 | best |  |
| cron-JuMP-1.13-ubuntu-latest | double_oscillator | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 0.000910923 | 5 | 0.01512 | 6.24867e-12 | best |  |
| cron-JuMP-1.13-ubuntu-latest | ducted_fan | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 1831.83 | 115 | 0.758462 | 7.15431e-8 | best |  |
| cron-JuMP-1.13-ubuntu-latest | electric_vehicle | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 1228.6 | 4 | 0.0341193 | 7.4027e-16 | best |  |
| cron-JuMP-1.13-ubuntu-latest | glider | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -1247.98 | 36 | 14.6656 | 8.79631e-13 | best |  |
| cron-JuMP-1.13-ubuntu-latest | insurance | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -2.05921 | 101 | 0.828019 | 0.0 | best |  |
| cron-JuMP-1.13-ubuntu-latest | jackson | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -0.191822 | 58 | 0.122867 | 1.38473e-13 | best |  |
| cron-JuMP-1.13-ubuntu-latest | moonlander | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 0.96181 | 69 | 1.59041 | 1.13344e-9 | best |  |
| cron-JuMP-1.13-ubuntu-latest | mountain_car | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 102.398 | 41 | 0.052293 | 0.0001619 | local |  |
| cron-JuMP-1.13-ubuntu-latest | robbins | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 19.4666 | 18 | 0.0290129 | 2.19502e-8 | best |  |
| cron-JuMP-1.13-ubuntu-latest | robot | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 9.14113 | 15 | 0.120383 | 3.44135e-8 | best |  |
| cron-JuMP-1.13-ubuntu-latest | rocket | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -1.01284 | 21 | 0.169606 | 6.57691e-16 | best |  |
| cron-JuMP-1.13-ubuntu-latest | space_shuttle | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -0.595875 | 116 | 3.58566 | 1.60427e-10 | best |  |
| cron-JuMP-1.13-ubuntu-latest | steering | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 0.554572 | 8 | 0.0863461 | 2.39554e-10 | best |  |
| cron-JuMP-1.13-ubuntu-latest | vanderpol | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 1.04781 | 3 | 0.0324276 | 3.60252e-15 | best |  |
| cron-JuMP-1.13-windows-latest | balanced_field | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 771.011 | 17 | 0.138637 | 1.47452e-16 | best |  |
| cron-JuMP-1.13-windows-latest | beam | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 8.88928 | 18 | 0.0360122 | 2.13633e-8 | best |  |
| cron-JuMP-1.13-windows-latest | bioreactor | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -12.9255 | 184 | 2.24455 | 0.000327787 | local |  |
| cron-JuMP-1.13-windows-latest | brachistochrone | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 1.80293 | 18 | 0.144273 | 0.0 | best |  |
| cron-JuMP-1.13-windows-latest | bryson_denham | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 4.00014 | 16 | 0.0369688 | 1.2409e-7 | best |  |
| cron-JuMP-1.13-windows-latest | cart_pendulum | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 1.74397 | 222 | 2.89591 | 6.20477e-12 | best |  |
| cron-JuMP-1.13-windows-latest | chain | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 5.06858 | 14 | 0.0994764 | 5.16935e-13 | best |  |
| cron-JuMP-1.13-windows-latest | dielectrophoretic_particle | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 7.81132 | 13 | 0.135054 | 2.27408e-16 | best |  |
| cron-JuMP-1.13-windows-latest | double_oscillator | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 0.000910923 | 5 | 0.0237618 | 6.24867e-12 | best |  |
| cron-JuMP-1.13-windows-latest | ducted_fan | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 1831.83 | 124 | 1.53168 | 7.14473e-8 | best |  |
| cron-JuMP-1.13-windows-latest | electric_vehicle | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 1228.6 | 4 | 0.0480834 | 7.4027e-16 | best |  |
| cron-JuMP-1.13-windows-latest | glider | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -1247.98 | 68 | 25.8487 | 3.64387e-15 | best |  |
| cron-JuMP-1.13-windows-latest | insurance | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -2.05921 | 95 | 1.25759 | 2.21305e-10 | best |  |
| cron-JuMP-1.13-windows-latest | jackson | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -0.191822 | 58 | 0.217955 | 1.38473e-13 | best |  |
| cron-JuMP-1.13-windows-latest | moonlander | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 0.96181 | 34 | 1.04728 | 3.37381e-12 | best |  |
| cron-JuMP-1.13-windows-latest | mountain_car | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 102.398 | 41 | 0.0870647 | 0.0001619 | local |  |
| cron-JuMP-1.13-windows-latest | robbins | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 19.4666 | 18 | 0.0480568 | 2.19502e-8 | best |  |
| cron-JuMP-1.13-windows-latest | robot | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 9.14113 | 15 | 0.17364 | 3.44135e-8 | best |  |
| cron-JuMP-1.13-windows-latest | rocket | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -1.01284 | 21 | 0.19587 | 8.76922e-16 | best |  |
| cron-JuMP-1.13-windows-latest | space_shuttle | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -0.595875 | 116 | 4.63177 | 1.60425e-10 | best |  |
| cron-JuMP-1.13-windows-latest | steering | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 0.554572 | 8 | 0.12527 | 2.39555e-10 | best |  |
| cron-JuMP-1.13-windows-latest | vanderpol | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 1.04781 | 3 | 0.10122 | 3.60252e-15 | best |  |
