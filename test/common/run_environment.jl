# Entry point of the subprocess that runs one test environment.
# Arguments: <directory of the test files> <name> [patterns...]
include(joinpath(@__DIR__, "options.jl"))
include(joinpath(@__DIR__, "solver_options.jl"))
include(joinpath(@__DIR__, "helpers.jl"))
include(joinpath(@__DIR__, "runner.jl"))

dir = ARGS[1]
name = ARGS[2]
patterns = String.(ARGS[3:end])

# A failing test makes the enclosing test set throw, hence a non-zero exit code.
LocalTestRunner.run_dir(dir; patterns=patterns, name=name)
