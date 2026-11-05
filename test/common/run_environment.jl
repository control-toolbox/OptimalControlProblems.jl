# Entry point of the subprocess that runs one test environment.
# Arguments: <directory of the test files> <name> [patterns...]
include(joinpath(@__DIR__, "options.jl"))
include(joinpath(@__DIR__, "configs.jl"))
include(joinpath(@__DIR__, "status.jl"))
include(joinpath(@__DIR__, "selection.jl"))

dir = ARGS[1]
name = ARGS[2]
problem_names, patterns = TestSelection.split_problems(String.(ARGS[3:end]))
TestStatus.select_problems!(problem_names)

# A failing test makes the enclosing test set throw, hence a non-zero exit code; the report
# of the statuses is written in any case.
cron = basename(dirname(dir)) == "cron"
try
    TestSelection.run_dir(dir; patterns=patterns, name=name)
finally
    TestStatus.write_report(cron ? "$name-cron" : name)
end
