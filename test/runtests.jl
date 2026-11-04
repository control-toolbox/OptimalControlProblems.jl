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
#   julia --project -e 'using Pkg; Pkg.test(; test_args=["JuMP", "cron", "*beam*"])'
#   julia --project -e 'using Pkg; Pkg.test(; test_args=["JuMP", "problem=beam,robot"])'
#
# `test_args` is a list of: environment names (none given: every environment except the
# opt-in ones below), the word `cron` (run `cron/<group>/test_*.jl` instead of
# `suite/<group>/test_*.jl`), and file patterns (`*` and `?` allowed, with or without
# `test_` and `.jl`);
# - `problem=a,b`: test only these problems (a name not in the catalogue is an error), for
#   instance the one you add; `core` and the environments are unchanged otherwise.
#
# Test files follow TST1: a module and an entry function `test_<name>()`.
# ==============================================================================

using Test: Test

include(joinpath(@__DIR__, "common", "options.jl"))
include(joinpath(@__DIR__, "common", "configs.jl"))
include(joinpath(@__DIR__, "common", "status.jl"))
include(joinpath(@__DIR__, "common", "runner.jl"))

const ROOT = dirname(@__DIR__)
const CORE = "core"

# Environments that run only when named (their label, or the cron): heavy dependencies.
const OPT_IN = ("ModelingToolkit",)

const SUITE = joinpath(@__DIR__, "suite")
const CRON = joinpath(@__DIR__, "cron")

"""
Names of the test environments: the groups of `suite/` holding a `Project.toml`.
"""
function environments()
    return sort([d for d in readdir(SUITE) if isfile(joinpath(SUITE, d, "Project.toml"))])
end

"""
Run the tests of one environment in a Julia subprocess: those of `suite/<name>`, or of
`cron/<name>` in cron mode, in the environment of `suite/<name>`. Its `Project.toml` is
copied to a temporary directory where the package under test is developed, so that no
`Manifest.toml` is written in the working tree. The code coverage flag of this process
is forwarded (`Base.julia_cmd`). Returns `true` on success.
"""
function run_environment(name::AbstractString; cron::Bool, patterns::Vector{String})
    tests = joinpath(cron ? CRON : SUITE, name)
    project = mktempdir()
    cp(joinpath(SUITE, name, "Project.toml"), joinpath(project, "Project.toml"))
    # `Pkg.test` restricts JULIA_LOAD_PATH (and may set JULIA_PROJECT) to its own
    # temporary environment: a subprocess inheriting them would ignore `--project`.
    julia = addenv(Base.julia_cmd(), "JULIA_LOAD_PATH" => nothing, "JULIA_PROJECT" => nothing)
    try
        setup = "using Pkg: Pkg; Pkg.develop(path=ARGS[1]); Pkg.instantiate()"
        run(`$julia --project=$project -e $setup $ROOT`)
        script = joinpath(@__DIR__, "common", "run_environment.jl")
        problem_args = isempty(problem_names) ? String[] : ["problem=" * join(problem_names, ",")]
        run(`$julia --project=$project $script $tests $name $problem_args $patterns`)
        return true
    catch err
        @error "Environment $name failed" exception = err
        return false
    finally
        rm(project; recursive=true, force=true)
    end
end

args = String.(ARGS)
cron = "cron" in args
problem_names, args = LocalTestRunner.split_problems(args)
TestStatus.select_problems!(problem_names)
rest = filter(!=("cron"), args)
available = environments()
selected, patterns = LocalTestRunner.select_environments(rest, available, OPT_IN; core=CORE)
# In cron mode, an environment without `cron/<name>/` has nothing to run: skip it (its
# environment is instantiated for nothing otherwise).
cron && filter!(name -> isdir(joinpath(CRON, name)), selected)

Test.@testset "OptimalControlProblems tests" verbose = Main.TestOptions.VERBOSE showtiming =
    Main.TestOptions.SHOWTIMING begin
    println("Environments: ", join([CORE; selected], ", "), cron ? " (cron)" : "")

    # `core` has no cron tests: in cron mode its directory does not exist and nothing runs.
    try
        LocalTestRunner.run_dir(
            joinpath(cron ? CRON : SUITE, CORE); patterns=patterns, name=CORE
        )
    finally
        TestStatus.write_report(CORE)
    end

    for name in selected
        Test.@testset "environment $name" begin
            Test.@test run_environment(name; cron=cron, patterns=patterns)
        end
    end
end
