# Cron report: JuMP

<sub>`CronJuMP.yml` · weekly (Tuesday 04:00 UTC) · Julia 1.13, Ubuntu + Windows, CPU</sub>

Generated: 2026-11-24T04:19:26+00:00
Run: https://github.com/control-toolbox/OptimalControlProblems.jl/actions/runs/45324289719
Commit: `fff425e8f25fe1cf3273152950cfce5f6e44e205`
Julia: `1.13`
Package version: `0.5.0`

| platform | test | problem | modelings | status | outcome |
| --- | --- | --- | --- | --- | --- |
| cron-JuMP-1.13-ubuntu-latest | test_convergence solver=ipopt | balanced_field | JuMP | valid | pass |
| cron-JuMP-1.13-ubuntu-latest | test_convergence solver=ipopt | beam | JuMP | valid | pass |
| cron-JuMP-1.13-ubuntu-latest | test_convergence solver=ipopt | bioreactor | JuMP | valid | pass |
| cron-JuMP-1.13-ubuntu-latest | test_convergence solver=ipopt | brachistochrone | JuMP | valid | pass |
| cron-JuMP-1.13-ubuntu-latest | test_convergence solver=ipopt | bryson_denham | JuMP | valid | pass |
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
| cron-JuMP-1.13-windows-latest | test_convergence solver=ipopt | bryson_denham | JuMP | valid | pass |
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
| cron-JuMP-1.13-ubuntu-latest | balanced_field | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 771.011 | 17 | 0.129117 | 2.33249e-10 | best |  |
| cron-JuMP-1.13-ubuntu-latest | beam | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 8.88928 | 18 | 0.0274528 | 4.74394e-8 | best |  |
| cron-JuMP-1.13-ubuntu-latest | bioreactor | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -12.9255 | 184 | 1.95194 | 0.000327787 | local |  |
| cron-JuMP-1.13-ubuntu-latest | brachistochrone | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 1.80293 | 18 | 0.127214 | 3.02973e-8 | best |  |
| cron-JuMP-1.13-ubuntu-latest | bryson_denham | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 4.00014 | 16 | 0.027632 | 0.0 | best |  |
| cron-JuMP-1.13-ubuntu-latest | cart_pendulum | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 1.74397 | 316 | 4.10542 | 6.96861e-7 | best |  |
| cron-JuMP-1.13-ubuntu-latest | chain | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 5.06858 | 14 | 0.0870066 | 5.25697e-16 | best |  |
| cron-JuMP-1.13-ubuntu-latest | dielectrophoretic_particle | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 7.81132 | 13 | 0.0528649 | 4.41642e-8 | best |  |
| cron-JuMP-1.13-ubuntu-latest | double_oscillator | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 0.000910923 | 5 | 0.0200075 | 6.24487e-12 | best |  |
| cron-JuMP-1.13-ubuntu-latest | ducted_fan | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 1831.83 | 115 | 1.13856 | 5.07582e-9 | best |  |
| cron-JuMP-1.13-ubuntu-latest | electric_vehicle | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 1228.6 | 4 | 0.107925 | 0.0 | best |  |
| cron-JuMP-1.13-ubuntu-latest | glider | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -1247.98 | 36 | 22.721 | 1.02109e-10 | best |  |
| cron-JuMP-1.13-ubuntu-latest | insurance | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -2.05921 | 101 | 1.32379 | 4.51907e-7 | best |  |
| cron-JuMP-1.13-ubuntu-latest | jackson | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -0.191822 | 58 | 0.185027 | 9.40053e-7 | best |  |
| cron-JuMP-1.13-ubuntu-latest | moonlander | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 0.96181 | 69 | 1.96102 | 9.14145e-7 | best |  |
| cron-JuMP-1.13-ubuntu-latest | mountain_car | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 102.398 | 41 | 0.0724882 | 0.000161895 | local |  |
| cron-JuMP-1.13-ubuntu-latest | robbins | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 19.4666 | 18 | 0.0386096 | 8.49109e-9 | best |  |
| cron-JuMP-1.13-ubuntu-latest | robot | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 9.14113 | 15 | 0.168266 | 1.62977e-7 | best |  |
| cron-JuMP-1.13-ubuntu-latest | rocket | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -1.01284 | 21 | 0.20044 | 1.71069e-6 | best |  |
| cron-JuMP-1.13-ubuntu-latest | space_shuttle | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -0.595875 | 116 | 4.49308 | 1.99979e-11 | best |  |
| cron-JuMP-1.13-ubuntu-latest | steering | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 0.554572 | 8 | 0.120256 | 0.0 | best |  |
| cron-JuMP-1.13-ubuntu-latest | vanderpol | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 1.04781 | 3 | 0.0384607 | 0.0 | best |  |
| cron-JuMP-1.13-windows-latest | balanced_field | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 771.011 | 17 | 0.222797 | 2.33249e-10 | best |  |
| cron-JuMP-1.13-windows-latest | beam | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 8.88928 | 18 | 0.0513218 | 4.74394e-8 | best |  |
| cron-JuMP-1.13-windows-latest | bioreactor | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -12.9255 | 184 | 2.80295 | 0.000327787 | local |  |
| cron-JuMP-1.13-windows-latest | brachistochrone | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 1.80293 | 18 | 0.202543 | 3.02973e-8 | best |  |
| cron-JuMP-1.13-windows-latest | bryson_denham | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 4.00014 | 16 | 0.0440224 | 0.0 | best |  |
| cron-JuMP-1.13-windows-latest | cart_pendulum | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 1.74397 | 222 | 3.47255 | 6.99009e-7 | best |  |
| cron-JuMP-1.13-windows-latest | chain | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 5.06858 | 14 | 0.151519 | 0.0 | best |  |
| cron-JuMP-1.13-windows-latest | dielectrophoretic_particle | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 7.81132 | 13 | 0.0958703 | 4.41642e-8 | best |  |
| cron-JuMP-1.13-windows-latest | double_oscillator | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 0.000910923 | 5 | 0.0330908 | 6.24487e-12 | best |  |
| cron-JuMP-1.13-windows-latest | ducted_fan | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 1831.83 | 124 | 2.0316 | 5.1716e-9 | best |  |
| cron-JuMP-1.13-windows-latest | electric_vehicle | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 1228.6 | 4 | 0.100078 | 0.0 | best |  |
| cron-JuMP-1.13-windows-latest | glider | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -1247.98 | 68 | 36.7639 | 1.01226e-10 | best |  |
| cron-JuMP-1.13-windows-latest | insurance | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -2.05921 | 95 | 1.50154 | 4.51685e-7 | best |  |
| cron-JuMP-1.13-windows-latest | jackson | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -0.191822 | 58 | 0.314942 | 9.40053e-7 | best |  |
| cron-JuMP-1.13-windows-latest | moonlander | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 0.96181 | 34 | 1.44246 | 9.15275e-7 | best |  |
| cron-JuMP-1.13-windows-latest | mountain_car | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 102.398 | 41 | 0.125085 | 0.000161895 | local |  |
| cron-JuMP-1.13-windows-latest | robbins | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 19.4666 | 18 | 0.0651406 | 8.49109e-9 | best |  |
| cron-JuMP-1.13-windows-latest | robot | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 9.14113 | 15 | 0.233289 | 1.62977e-7 | best |  |
| cron-JuMP-1.13-windows-latest | rocket | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -1.01284 | 21 | 0.287877 | 1.71069e-6 | best |  |
| cron-JuMP-1.13-windows-latest | space_shuttle | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | -0.595875 | 116 | 6.24776 | 2.00003e-11 | best |  |
| cron-JuMP-1.13-windows-latest | steering | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 0.554572 | 8 | 0.179661 | 4.00389e-16 | best |  |
| cron-JuMP-1.13-windows-latest | vanderpol | JuMP | solver=ipopt | converged | LOCALLY_SOLVED | 1.04781 | 3 | 0.0656095 | 0.0 | best |  |
