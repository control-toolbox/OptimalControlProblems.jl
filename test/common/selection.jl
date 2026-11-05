# Selection of what the dispatcher runs: environments, mode (`cron`, `convergence`) and
# `problem=` arguments.
#
# The discovery of the test files, their selection by patterns and their execution
# (`test_<name>()` per file, TST1) are `CTBase.DevTools.run_tests`; what it does not know is
# local: the environments (one Julia process each), the opt-in ones, the mode and `problem=`.
module TestSelection

using CTBase: CTBase
using Test: Test

"""
    run_dir(dir; patterns=String[], name=basename(dir)) -> Bool

Run the test files of `dir` selected by `patterns` with `CTBase.DevTools.run_tests`: it
includes each file in `Main` and calls its entry function `test_<name>()` (TST1). Returns
`false` when `dir` does not exist (`core` has no `cron/` directory), `true` otherwise.
"""
function run_dir(
    dir::AbstractString;
    patterns::AbstractVector{<:AbstractString}=String[],
    name::AbstractString=basename(dir),
)
    isdir(dir) || return false
    verbose = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
    showtiming = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true
    CTBase.DevTools.run_tests(;
        args=patterns, testset_name=name, test_dir=String(dir), verbose, showtiming
    )
    return true
end

"""
    select_environments(args, available, opt_in; core="core")

Split the command-line `args` into the environments to run and the file patterns.
Names of `available` environments (and `core`) are environment names; the rest are
patterns. With no environment name, every available environment except the `opt_in`
ones runs; naming only `core` runs no environment (`core` always runs in-process).
"""
function select_environments(args, available, opt_in; core::AbstractString="core")
    named = filter(a -> a == core || a in available, args)
    patterns = String[a for a in args if !(a in named)]
    environments = if isempty(named)
        String[e for e in available if !(e in opt_in)]
    else
        String[e for e in named if e in available]
    end
    return environments, patterns
end

"""
    split_problems(args) -> (problems, rest)

Take the `problem=a,b` arguments out of `args`: the names they list (as strings) and the
other arguments. Several `problem=` arguments add up.
"""
function split_problems(args)
    names = String[]
    rest = String[]
    for arg in args
        if startswith(arg, "problem=")
            append!(names, filter(!isempty, split(chopprefix(arg, "problem="), ',')))
        else
            push!(rest, arg)
        end
    end
    return unique!(names), rest
end

"""
    split_mode(args) -> (mode, rest)

Take the words `cron` and `convergence` out of `args`. The mode is `:suite` (neither:
`suite/<group>`), `:cron` (`cron/<group>` only, as the cron workflows run it) or
`:convergence` (`suite/<group>` then `cron/<group>`: the convergence report on demand,
#360). Both words together are an error.
"""
function split_mode(args)
    cron = "cron" in args
    convergence = "convergence" in args
    cron && convergence && throw(
        ArgumentError(
            "`cron` and `convergence` cannot be combined: `cron` runs the cron directories only, `convergence` the suite and the cron directories",
        ),
    )
    mode = cron ? :cron : (convergence ? :convergence : :suite)
    return mode, String[a for a in args if !(a in ("cron", "convergence"))]
end

"""
    directories(group, suite, cron; mode) -> Vector{String}

The test directories of `group` for `mode` (see [`split_mode`](@ref)), among `suite/<group>`
and `cron/<group>`, in this order; only those that exist.
"""
function directories(
    group::AbstractString, suite::AbstractString, cron::AbstractString; mode::Symbol
)
    roots = mode === :suite ? (suite,) : (mode === :cron ? (cron,) : (suite, cron))
    return String[joinpath(r, group) for r in roots if isdir(joinpath(r, group))]
end

end # module TestSelection
