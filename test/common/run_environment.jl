# Entry point of the subprocess that runs one test environment.
# Arguments: <directories of the test files, separated by `|`> <name> <report> [patterns...]
include(joinpath(@__DIR__, "options.jl"))
include(joinpath(@__DIR__, "configs.jl"))
include(joinpath(@__DIR__, "status.jl"))
include(joinpath(@__DIR__, "selection.jl"))

dirs = split(ARGS[1], '|')
name = ARGS[2]
report = ARGS[3]
problem_names, patterns = TestSelection.split_problems(String.(ARGS[4:end]))
TestStatus.select_problems!(problem_names)

# A failing test makes the enclosing test set throw, hence a non-zero exit code; the report
# of the statuses is written in any case.
# Every directory runs even if an earlier one fails; the first failure is rethrown at the end.
failure = nothing
try
    for dir in dirs
        label = basename(dirname(dir)) == "cron" ? "$name (cron)" : name
        try
            TestSelection.run_dir(dir; patterns=patterns, name=label)
        catch err
            global failure = something(failure, err)
        end
    end
finally
    TestStatus.write_report(report)
end
isnothing(failure) || throw(failure)
