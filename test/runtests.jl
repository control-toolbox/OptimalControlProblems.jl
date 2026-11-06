# ==============================================================================
# OptimalControlProblems test runner
# ==============================================================================
#
# Layout (TST1, as CTFlows.jl): `suite/<group>/test_<name>.jl` for the tests of every
# run, `cron/<group>/test_<name>.jl` for the slow ones (cron mode).
#
# One environment per modeling: each group of `suite/` with a `Project.toml` (JuMP,
# OptimalControl, ...) is run in its own Julia process, so that an extension is shown to
# load alone and heavy dependencies stay isolated; the slow tests of `cron/<group>/` run
# in the environment of `suite/<group>/`. `suite/core/` has no extension and runs in this
# process, always.
#
# ## Running tests
#
#   julia --project -e 'using Pkg; Pkg.test()'                       # core + all environments
#   julia --project -e 'using Pkg; Pkg.test(; test_args=["JuMP"])'   # core + one environment
#   julia --project -e 'using Pkg; Pkg.test(; test_args=["cron"])'   # slow tests (cron/ dirs)
#   julia --project -e 'using Pkg; Pkg.test(; test_args=["convergence"])' # suite + cron/ dirs
#   julia --project -e 'using Pkg; Pkg.test(; test_args=["robustness"])'  # robustness/ dirs
#   julia --project -e 'using Pkg; Pkg.test(; test_args=["JuMP", "cron", "*beam*"])'
#   julia --project -e 'using Pkg; Pkg.test(; test_args=["JuMP", "problem=beam,robot"])'
#
# `test_args` is a list of: environment names (none given: every environment except the
# opt-in ones below), the word `cron` (run `cron/<group>/test_*.jl` instead of
# `suite/<group>/test_*.jl`), the word `convergence` (run both: the convergence report,
# off by default, #360) or the word `robustness` (run `robustness/<group>/test_*.jl` only:
# the robustness report of the monthly cron, #378), and file patterns (`*` and `?` allowed, with or without
# `test_` and `.jl`);
# - `problem=a,b`: test only these problems (a name not in the catalogue is an error), for
#   instance the one you add; `core` and the environments are unchanged otherwise.
#
# Test files follow TST1: a module and an entry function `test_<name>()`. Their discovery,
# their selection by the patterns and their execution are `CTBase.DevTools.run_tests`; the
# environments, `cron` and `problem=` are local (`common/selection.jl`).
# ==============================================================================

using Test: Test

include(joinpath(@__DIR__, "common", "options.jl"))
include(joinpath(@__DIR__, "common", "configs.jl"))
include(joinpath(@__DIR__, "common", "status.jl"))
include(joinpath(@__DIR__, "common", "selection.jl"))

const ROOT = dirname(@__DIR__)
const CORE = "core"

# Environments that run only when named (their label, or the cron): heavy dependencies.
const OPT_IN = ("ModelingToolkit", "GPU")

const SUITE = joinpath(@__DIR__, "suite")
const CRON = joinpath(@__DIR__, "cron")
const ROBUSTNESS = joinpath(@__DIR__, "robustness")

"""
Names of the test environments: the groups of `suite/` holding a `Project.toml`.
"""
function environments()
    return sort([d for d in readdir(SUITE) if isfile(joinpath(SUITE, d, "Project.toml"))])
end

"""
Run the tests of one environment in a Julia subprocess: those of the directories `dirs`
(`suite/<name>`, `cron/<name>` or both, see `TestSelection.directories`), in the environment
of `suite/<name>`, the report being written as `test/reports/<report>.md`. Its `Project.toml` is
copied to a temporary directory where the package under test is developed, so that no
`Manifest.toml` is written in the working tree; the directory is removed afterwards, except in
GitHub Actions (see below). The code coverage flag of this process is forwarded
(`Base.julia_cmd`). Returns `true` on success.
"""
function run_environment(
    name::AbstractString; dirs::Vector{String}, report::AbstractString, patterns::Vector{String}
)
    # In GitHub Actions the project is kept (see `finally`), also past the exit of Julia.
    keep = get(ENV, "GITHUB_ACTIONS", "false") == "true"
    project = mktempdir(; cleanup=!keep)
    cp(joinpath(SUITE, name, "Project.toml"), joinpath(project, "Project.toml"))
    # `Pkg.test` restricts JULIA_LOAD_PATH (and may set JULIA_PROJECT) to its own
    # temporary environment: a subprocess inheriting them would ignore `--project`.
    julia = addenv(
        Base.julia_cmd(), "JULIA_LOAD_PATH" => nothing, "JULIA_PROJECT" => nothing
    )
    try
        setup = "using Pkg: Pkg; Pkg.develop(path=ARGS[1]); Pkg.instantiate()"
        run(`$julia --project=$project -e $setup $ROOT`)
        script = joinpath(@__DIR__, "common", "run_environment.jl")
        problem_args =
            isempty(problem_names) ? String[] : ["problem=" * join(problem_names, ",")]
        tests = join(dirs, "|")
        run(`$julia --project=$project $script $tests $name $report $problem_args $patterns`)
        return true
    catch err
        @error "Environment $name failed" exception = err
        return false
    finally
        # Kept in GitHub Actions: the `Pkg.gc` that `julia-actions/cache` runs before saving the
        # depot deletes the packages used only by a manifest that no longer exists, so the
        # packages of every environment, installed and precompiled here, would be missing from
        # the cache and rebuilt by the next run. The runner is discarded at the end of the job.
        keep || rm(project; recursive=true, force=true)
    end
end

mode, args = TestSelection.split_mode(String.(ARGS))
problem_names, rest = TestSelection.split_problems(args)
TestStatus.select_problems!(problem_names)
available = environments()
selected, patterns = TestSelection.select_environments(rest, available, OPT_IN; core=CORE)
dirs(name) = TestSelection.directories(name, SUITE, CRON, ROBUSTNESS; mode=mode)
# An environment with no directory to run in this mode (no `cron/<name>/` in cron mode) is
# skipped: its environment would be instantiated for nothing.
filter!(name -> !isempty(dirs(name)), selected)

Test.@testset "OptimalControlProblems tests" verbose = Main.TestOptions.VERBOSE showtiming =
    Main.TestOptions.SHOWTIMING begin
    println("Environments: ", join([CORE; selected], ", "), mode === :suite ? "" : " ($mode)")

    # `core` has no cron tests: in cron mode it has no directory and nothing runs.
    try
        for dir in dirs(CORE)
            TestSelection.run_dir(dir; patterns=patterns, name=CORE)
        end
    finally
        TestStatus.write_report(CORE)
    end

    for name in selected
        Test.@testset "environment $name" begin
            report = mode in (:cron, :robustness) ? "$name-$mode" : name
            Test.@test run_environment(name; dirs=dirs(name), report=report, patterns=patterns)
        end
    end
end
