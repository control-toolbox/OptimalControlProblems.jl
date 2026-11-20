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

Take the words `cron`, `convergence` and `robustness` out of `args`. The mode is `:suite`
(none: `suite/<group>`), `:cron` (`cron/<group>` only, as the cron workflows run it),
`:convergence` (`suite/<group>` then `cron/<group>`: the convergence report on demand,
#360) or `:robustness` (`robustness/<group>` only: the robustness report, #378). Two words
together are an error.
"""
function split_mode(args)
    words = [w for w in MODES if w in args]
    length(words) > 1 && throw(
        ArgumentError(
            "`$(join(words, "` and `"))` cannot be combined: `cron` runs the cron directories only, `convergence` the suite and the cron directories, `robustness` the robustness directories only",
        ),
    )
    mode = isempty(words) ? :suite : Symbol(only(words))
    return mode, String[a for a in args if !(a in MODES)]
end

const MODES = ("cron", "convergence", "robustness")

# The time a problem may take in a rotation (#435) before its process is stopped, by default.
const DEFAULT_TIMEOUT = 2700.0

"""
    split_budget(args) -> (budget, rest)

Take the `budget=<seconds>` and `timeout=<seconds>` arguments out of `args` (#435). With a
budget, the problems of an environment run one per process, in the order of `problem=`,
while the budget lasts (see [`run_within_budget`](@ref)); `timeout` is the time a problem
may take before its process is stopped ([`DEFAULT_TIMEOUT`](@ref) by default). `budget` is
`nothing` without `budget=`, a `NamedTuple` `(seconds, timeout)` otherwise.
"""
function split_budget(args)
    values = Dict{String,Float64}()
    rest = String[]
    for arg in args
        key = findfirst(k -> startswith(arg, k * "="), ("budget", "timeout"))
        if isnothing(key)
            push!(rest, arg)
            continue
        end
        name = ("budget", "timeout")[key]
        value = tryparse(Float64, chopprefix(arg, name * "="))
        (isnothing(value) || value <= 0) &&
            throw(ArgumentError("`$arg`: $name must be a positive number of seconds"))
        values[name] = value
    end
    haskey(values, "budget") || return nothing, rest
    return (seconds=values["budget"], timeout=get(values, "timeout", DEFAULT_TIMEOUT)), rest
end

"""
    run_within_budget(f, problems, seconds; clock=time) -> (outcomes, skipped)

Call `f(problem)` for the `problems` in their order while less than `seconds` have passed
since the start (#435): the first one always runs, a problem starts only if the budget is
not spent. Returns the `problem => f(problem)` pairs, in order, and the problems left for a
later run.
"""
function run_within_budget(f, problems, seconds::Real; clock=time)
    start = clock()
    outcomes = Pair{String,Symbol}[]
    skipped = String[]
    for problem in problems
        if clock() - start >= seconds
            push!(skipped, problem)
        else
            push!(outcomes, problem => f(problem))
        end
    end
    return outcomes, skipped
end

"""
    run_process(cmd; timeout) -> Symbol

Run `cmd`, its output shown, and stop it after `timeout` seconds. Returns `:success`,
`:failed` (a non-zero exit code: a test failed), `:crashed` (stopped by a signal, e.g. out of
memory) or `:timeout`.
"""
function run_process(cmd::Cmd; timeout::Real)
    process = run(ignorestatus(cmd); wait=false)
    start = time()
    while process_running(process)
        if time() - start > timeout
            kill(process)
            wait(process)
            return :timeout
        end
        sleep(1)
    end
    success(process) && return :success
    return process.termsignal == 0 ? :failed : :crashed
end

"""
    directories(group, suite, cron, robustness; mode) -> Vector{String}

The test directories of `group` for `mode` (see [`split_mode`](@ref)), among `suite/<group>`,
`cron/<group>` and `robustness/<group>`, in this order; only those that exist.
"""
function directories(
    group::AbstractString,
    suite::AbstractString,
    cron::AbstractString,
    robustness::AbstractString;
    mode::Symbol,
)
    roots = Dict(
        :suite => (suite,),
        :cron => (cron,),
        :convergence => (suite, cron),
        :robustness => (robustness,),
    )[mode]
    return String[joinpath(r, group) for r in roots if isdir(joinpath(r, group))]
end

end # module TestSelection
