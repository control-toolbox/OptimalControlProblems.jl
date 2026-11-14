# Cron report: JuMP

<sub>`CronJuMP.yml` · weekly (Tuesday 04:00 UTC) · Julia 1.13, Ubuntu + Windows, CPU</sub>

Generated: 2026-11-10T04:18:17+00:00
Run: https://github.com/control-toolbox/OptimalControlProblems.jl/actions/runs/42753891826
Commit: `bc717912017e10455bda33acdec26de43c90d5aa`
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
| cron-JuMP-1.13-ubuntu-latest | balanced_field | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 771.011 | 17 | 0.140345 | 2.33435e-11 | best |  |
| cron-JuMP-1.13-ubuntu-latest | beam | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 8.88928 | 18 | 0.0295845 | 1.78929e-10 | best |  |
| cron-JuMP-1.13-ubuntu-latest | bioreactor | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -12.9255 | 184 | 1.82486 | 0.000327787 | local |  |
| cron-JuMP-1.13-ubuntu-latest | brachistochrone | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 1.80293 | 18 | 0.135231 | 3.02973e-8 | best |  |
| cron-JuMP-1.13-ubuntu-latest | bryson_denham | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 4.00014 | 16 | 0.0260215 | 1.06404e-7 | best |  |
| cron-JuMP-1.13-ubuntu-latest | cart_pendulum | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 1.74397 | 316 | 2.94113 | 1.26131e-7 | best |  |
| cron-JuMP-1.13-ubuntu-latest | chain | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 5.06858 | 14 | 0.0950294 | 8.76161e-16 | best |  |
| cron-JuMP-1.13-ubuntu-latest | dielectrophoretic_particle | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 7.81132 | 13 | 0.144216 | 4.41642e-8 | best |  |
| cron-JuMP-1.13-ubuntu-latest | double_oscillator | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 0.000910923 | 5 | 0.0208343 | 6.24689e-12 | best |  |
| cron-JuMP-1.13-ubuntu-latest | ducted_fan | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 1831.83 | 115 | 1.37901 | 5.07477e-9 | best |  |
| cron-JuMP-1.13-ubuntu-latest | electric_vehicle | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 1228.6 | 4 | 0.0478518 | 7.4027e-16 | best |  |
| cron-JuMP-1.13-ubuntu-latest | glider | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -1247.98 | 36 | 34.7063 | 2.35795e-12 | best |  |
| cron-JuMP-1.13-ubuntu-latest | insurance | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -2.05921 | 101 | 1.05809 | 4.51907e-7 | best |  |
| cron-JuMP-1.13-ubuntu-latest | jackson | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -0.191822 | 58 | 0.190774 | 3.791e-14 | best |  |
| cron-JuMP-1.13-ubuntu-latest | moonlander | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 0.96181 | 69 | 1.80747 | 9.14145e-7 | best |  |
| cron-JuMP-1.13-ubuntu-latest | mountain_car | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 102.398 | 41 | 0.0812807 | 0.000161897 | local |  |
| cron-JuMP-1.13-ubuntu-latest | robbins | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 19.4666 | 18 | 0.0415428 | 1.2297e-8 | best |  |
| cron-JuMP-1.13-ubuntu-latest | robot | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 9.14113 | 15 | 0.163893 | 1.62977e-7 | best |  |
| cron-JuMP-1.13-ubuntu-latest | rocket | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -1.01284 | 21 | 0.181981 | 1.71069e-6 | best |  |
| cron-JuMP-1.13-ubuntu-latest | space_shuttle | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -0.595875 | 116 | 3.6739 | 1.99964e-11 | best |  |
| cron-JuMP-1.13-ubuntu-latest | steering | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 0.554572 | 8 | 0.112504 | 2.00195e-16 | best |  |
| cron-JuMP-1.13-ubuntu-latest | vanderpol | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 1.04781 | 3 | 0.126638 | 6.35738e-16 | best |  |
| cron-JuMP-1.13-windows-latest | balanced_field | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 771.011 | 17 | 0.174661 | 2.33434e-11 | best |  |
| cron-JuMP-1.13-windows-latest | beam | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 8.88928 | 18 | 0.0480328 | 8.25363e-12 | best |  |
| cron-JuMP-1.13-windows-latest | bioreactor | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -12.9255 | 198 | 3.17106 | 0.000327787 | local |  |
| cron-JuMP-1.13-windows-latest | brachistochrone | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 1.80293 | 18 | 0.1997 | 3.02973e-8 | best |  |
| cron-JuMP-1.13-windows-latest | bryson_denham | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 4.00014 | 16 | 0.0418832 | 1.06404e-7 | best |  |
| cron-JuMP-1.13-windows-latest | cart_pendulum | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 1.74397 | 441 | 6.92649 | 1.21986e-7 | best |  |
| cron-JuMP-1.13-windows-latest | chain | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 5.06858 | 14 | 0.133468 | 3.50465e-16 | best |  |
| cron-JuMP-1.13-windows-latest | dielectrophoretic_particle | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 7.81132 | 13 | 0.0737726 | 4.41642e-8 | best |  |
| cron-JuMP-1.13-windows-latest | double_oscillator | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 0.000910923 | 5 | 0.0310458 | 6.24677e-12 | best |  |
| cron-JuMP-1.13-windows-latest | ducted_fan | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 1831.83 | 120 | 1.8566 | 7.66177e-8 | best |  |
| cron-JuMP-1.13-windows-latest | electric_vehicle | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 1228.6 | 4 | 0.0616019 | 7.4027e-16 | best |  |
| cron-JuMP-1.13-windows-latest | glider | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -1247.98 | 40 | 28.7186 | 1.3741e-12 | best |  |
| cron-JuMP-1.13-windows-latest | insurance | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -2.05921 | 95 | 1.5465 | 4.51681e-7 | best |  |
| cron-JuMP-1.13-windows-latest | jackson | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -0.191822 | 58 | 0.298072 | 1.00563e-13 | best |  |
| cron-JuMP-1.13-windows-latest | moonlander | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 0.96181 | 94 | 3.49402 | 8.95337e-7 | best |  |
| cron-JuMP-1.13-windows-latest | mountain_car | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 102.398 | 41 | 0.115786 | 0.000161897 | local |  |
| cron-JuMP-1.13-windows-latest | robbins | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 19.4666 | 18 | 0.0677189 | 1.2297e-8 | best |  |
| cron-JuMP-1.13-windows-latest | robot | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 9.14113 | 15 | 0.254584 | 1.62977e-7 | best |  |
| cron-JuMP-1.13-windows-latest | rocket | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -1.01284 | 21 | 0.342681 | 1.71069e-6 | best |  |
| cron-JuMP-1.13-windows-latest | space_shuttle | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -0.595875 | 116 | 5.52267 | 1.99958e-11 | best |  |
| cron-JuMP-1.13-windows-latest | steering | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 0.554572 | 8 | 0.149798 | 2.00195e-16 | best |  |
| cron-JuMP-1.13-windows-latest | vanderpol | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 1.04781 | 3 | 0.0522316 | 6.35738e-16 | best |  |
