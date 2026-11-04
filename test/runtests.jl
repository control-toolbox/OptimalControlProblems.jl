# ==============================================================================
# OptimalControlProblems test runner
# ==============================================================================
#
# One environment per modeling: each sub-directory of `test/` with a `Project.toml`
# (JuMP, OptimalControl, ...) is run in its own Julia process, so that an extension is
# shown to load alone and heavy dependencies stay isolated. `core/` has no extension
# and runs in this process, always.
#
# ## Running tests
#
#   julia --project -e 'using Pkg; Pkg.test()'                       # core + all environments
#   julia --project -e 'using Pkg; Pkg.test(; test_args=["JuMP"])'   # core + one environment
#   julia --project -e 'using Pkg; Pkg.test(; test_args=["cron"])'   # slow tests (cron/ dirs)
#   julia --project -e 'using Pkg; Pkg.test(; test_args=["JuMP", "cron", "*beam*"])'
#
# `test_args` is a list of: environment names (none given: every environment except the
# opt-in ones below), the word `cron` (run `cron/test_*.jl` instead of `test_*.jl`),
# and file patterns (`*` and `?` allowed, with or without `test_` and `.jl`).
#
# Test files follow TST1: a module and an entry function `test_<name>()`.
# ==============================================================================

using Test: Test

include(joinpath(@__DIR__, "common", "options.jl"))
include(joinpath(@__DIR__, "common", "solver_options.jl"))
include(joinpath(@__DIR__, "common", "helpers.jl"))
include(joinpath(@__DIR__, "common", "runner.jl"))

const ROOT = dirname(@__DIR__)
const CORE = "core"

# Environments that run only when named (their label, or the cron): heavy dependencies.
const OPT_IN = ("ModelingToolkit",)

"""
Names of the test environments: the sub-directories of `test/` holding a `Project.toml`.
"""
function environments()
    return sort([
        d for d in readdir(@__DIR__) if isfile(joinpath(@__DIR__, d, "Project.toml"))
    ])
end

"""
Run the tests of one environment in a Julia subprocess. The `Project.toml` of the
environment is copied to a temporary directory, the package under test is developed
into it, so that no `Manifest.toml` is written in the working tree. The code coverage
flag of this process is forwarded (`Base.julia_cmd`). Returns `true` on success.
"""
function run_environment(name::AbstractString; cron::Bool, patterns::Vector{String})
    source = joinpath(@__DIR__, name)
    project = mktempdir()
    cp(joinpath(source, "Project.toml"), joinpath(project, "Project.toml"))
    # `Pkg.test` restricts JULIA_LOAD_PATH (and may set JULIA_PROJECT) to its own
    # temporary environment: a subprocess inheriting them would ignore `--project`.
    julia = addenv(Base.julia_cmd(), "JULIA_LOAD_PATH" => nothing, "JULIA_PROJECT" => nothing)
    try
        setup = "using Pkg: Pkg; Pkg.develop(path=ARGS[1]); Pkg.instantiate()"
        run(`$julia --project=$project -e $setup $ROOT`)
        script = joinpath(@__DIR__, "common", "run_environment.jl")
        run(`$julia --project=$project $script $source $cron $patterns`)
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
rest = filter(!=("cron"), args)
available = environments()
named = filter(a -> a == CORE || a in available, rest)
patterns = filter(a -> !(a in named), rest)
selected = if any(in(available), named)
    filter(in(available), named)
else
    filter(e -> !(e in OPT_IN), available)
end

Test.@testset "OptimalControlProblems tests" verbose = Main.TestOptions.VERBOSE showtiming =
    Main.TestOptions.SHOWTIMING begin
    println("Environments: ", join([CORE; selected], ", "), cron ? " (cron)" : "")

    LocalTestRunner.run_dir(joinpath(@__DIR__, CORE); cron=cron, patterns=patterns)

    for name in selected
        Test.@testset "environment $name" begin
            Test.@test run_environment(name; cron=cron, patterns=patterns)
        end
    end
end
