# Entry point of the subprocess that runs one test environment.
# Arguments: <environment directory> <cron: true|false> [patterns...]
include(joinpath(@__DIR__, "options.jl"))
include(joinpath(@__DIR__, "solver_options.jl"))
include(joinpath(@__DIR__, "helpers.jl"))
include(joinpath(@__DIR__, "runner.jl"))

dir = ARGS[1]
cron = parse(Bool, ARGS[2])
patterns = String.(ARGS[3:end])

# A failing test makes the enclosing test set throw, hence a non-zero exit code.
LocalTestRunner.run_dir(dir; cron=cron, patterns=patterns)
