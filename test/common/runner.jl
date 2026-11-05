# Minimal test runner, provisional.
#
# Same interface as `CTBase.run_tests` of CTBase >= 0.30 (file discovery, selection by
# patterns, `cron` mode, one entry function `test_<name>()` per file, TST1), which this
# package cannot use yet: CTDirect 0.17 and CTModels 0.6 pin CTBase 0.16. The port to
# OptimalControl v2 (#283) replaces this file by `CTBase.run_tests`.
module LocalTestRunner

using Test: Test

"""
    test_files(dir) -> Vector{String}

Names of the test files of `dir` (`test_*.jl`), sorted. A missing directory gives an
empty list.
"""
function test_files(dir::AbstractString)
    isdir(dir) || return String[]
    files = filter(f -> startswith(f, "test_") && endswith(f, ".jl"), readdir(dir))
    return sort(files)
end

"""
    glob_regex(pattern) -> Regex

Regular expression matching the whole string for a glob `pattern`, where `*` matches any
sequence of characters and `?` any single character.
"""
function glob_regex(pattern::AbstractString)
    escaped = replace(pattern, r"[.+^$(){}|\[\]\\]" => s"\\\0")
    escaped = replace(escaped, "*" => ".*", "?" => ".")
    return Regex("^" * escaped * "\$")
end

"""
    selected(file, patterns) -> Bool

`true` if `patterns` is empty, or if one pattern matches the name of `file` with or
without the `test_` prefix and the `.jl` extension (`test_runner.jl`, `test_runner`,
`runner`, `*run*`).
"""
function selected(file::AbstractString, patterns::AbstractVector{<:AbstractString})
    isempty(patterns) && return true
    stem = first(splitext(file))
    names = (file, stem, stem[(length("test_") + 1):end])
    return any(p -> any(n -> occursin(glob_regex(p), n), names), patterns)
end

"""
    run_dir(dir; patterns=String[], name=basename(dir)) -> Bool

Include and run the selected test files of `dir` in `Main`.
Each file defines `test_<name>()` in `Main` (TST1). Returns `false` when no file was
selected, `true` otherwise; a failing test makes the enclosing test set fail.
"""
function run_dir(
    dir::AbstractString;
    patterns::AbstractVector{<:AbstractString}=String[],
    name::AbstractString=basename(dir),
)
    files = filter(f -> selected(f, patterns), test_files(dir))
    if isempty(files)
        @info "No test selected in $name" patterns
        return false
    end
    verbose = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
    showtiming = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true
    Test.@testset "$name" verbose = verbose showtiming = showtiming begin
        for file in files
            entry = Symbol(first(splitext(file)))
            Base.include(Main, joinpath(dir, file))
            Test.@testset "$entry" verbose = verbose begin
                Base.invokelatest(Base.invokelatest(getfield, Main, entry))
            end
        end
    end
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

end # module LocalTestRunner
