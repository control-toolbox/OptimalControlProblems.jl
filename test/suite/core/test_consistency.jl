module TestConsistency

using Test: Test
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

const OCP = OptimalControlProblems
const ROOT = pkgdir(OptimalControlProblems)
const EXT = joinpath(ROOT, "ext")

# Directories of ext/ that hold one file per problem, and the extension of their files.
const DATA_DIRECTORIES = ("MetaData" => ".jl", "Descriptions" => ".md")

names_in(dir, extension) = sort!([Symbol(first(splitext(f))) for f in readdir(dir) if endswith(f, extension)])

"""Files of the problem `problem` in `ext/` (model files, metadata, description)."""
function test_files()
    Test.@testset "files" verbose = VERBOSE begin
        problems = OCP.problems(; status=:all)
        # metadata and descriptions: one file each per problem, and nothing else
        for (dir, extension) in DATA_DIRECTORIES
            Test.@test names_in(joinpath(EXT, dir), extension) == problems
        end
        # model directories: exactly the directories of the modeling table
        model_dirs = sort!([d for d in readdir(EXT) if isdir(joinpath(EXT, d)) && endswith(d, "Models")])
        Test.@test model_dirs == sort!([spec.directory for spec in OCP.MODELING_SPECS])
        for spec in OCP.MODELING_SPECS
            files = names_in(joinpath(EXT, spec.directory), ".jl")
            # every model file belongs to a problem that has metadata
            Test.@test files ⊆ problems
            # and the catalogue lists exactly the problems having a model file
            Test.@test files == OCP.problems(spec.backend(); status=:all)
        end
        # every problem has at least one modeling
        for problem in problems
            Test.@test any(spec -> isfile(joinpath(EXT, spec.directory, "$problem.jl")), OCP.MODELING_SPECS)
        end
    end
end

"""Required and optional fields of the metadata, with their types; statuses."""
function test_schema()
    Test.@testset "metadata schema" verbose = VERBOSE begin
        schema = OCP.METADATA_SCHEMA
        for problem in OCP.problems(; status=:all)
            data = OCP.problem_metadata(problem)
            for (key, T) in pairs(schema.required)
                Test.@test haskey(data, key) && data[key] isa T
            end
            for (key, value) in data
                haskey(schema.required, key) && continue
                T = get(schema.optional, key, get(schema.generated, key, Union{}))
                Test.@test value isa T
            end
            # a status is declared exactly for the modelings having a model file
            statuses = data[:status]
            for spec in OCP.MODELING_SPECS
                has_file = isfile(joinpath(EXT, spec.directory, "$problem.jl"))
                Test.@test haskey(statuses, spec.name) == has_file
            end
            Test.@test all(in((:draft, :valid)), values(statuses))
        end
    end
end

"""
Every problem `:valid` somewhere has a reference: another modeling where it is `:valid`, or
a reference objective at its default grid (#375).
"""
function test_comparison_rule()
    Test.@testset "comparison rule" verbose = VERBOSE begin
        for problem in OCP.problems(; status=:all)
            valid = [name for (name, s) in pairs(OCP.status(problem)) if s === :valid]
            isempty(valid) && continue
            Test.@test length(valid) >= 2 || !isnothing(OCP.reference_objective(problem))
        end
    end
end

"""
The reference objectives (#375) match the problem: computed at its default grid with its
default parameters, otherwise they are stale and must be computed again
(`tools/references/update.jl`). A problem `:valid` somewhere without a reference objective
is reported, not failed (the comparison rule fails when it has no other reference).
"""
function test_reference_objectives()
    Test.@testset "reference objectives" verbose = VERBOSE begin
        for problem in OCP.problems(; status=:all)
            data = OCP.problem_metadata(problem)
            references = get(data, :reference_objective, (;))
            for (scheme, entry) in pairs(references)
                Test.@test entry.grid_size == data[:grid_size]
                Test.@test entry.parameters == repr(data[:parameters])
                Test.@test entry.source in (:computed,)
                Test.@test isfinite(entry.value)
                # known solutions (#378): best first, distinct, the reference is the best
                Test.@test first(entry.solutions) == entry.value
                Test.@test issorted(entry.solutions)
                Test.@test entry.agreement == (length(entry.solutions) == 1)
                kind(x) = OCP.solution_kind(problem, x; grid_size=entry.grid_size, scheme=scheme)
                Test.@test kind(entry.value) === :best
                Test.@test all(s -> kind(s) in (:best, :local), entry.solutions)
                Test.@test all(s -> kind(s) === :local, entry.solutions[2:end])
                far = maximum(abs, entry.solutions) * 10 + 1
                Test.@test kind(far) === :new
            end
            valid = [name for (name, s) in pairs(OCP.status(problem)) if s === :valid]
            (isempty(valid) || !isnothing(OCP.reference_objective(problem))) && continue
            Main.TestStatus.report!(
                "reference objectives", problem, join(valid, ", "), :valid, "no reference objective",
            )
        end
    end
end

# Files where a problem name must never appear: the problem list is derived from files.
function source_files()
    files = String[]
    for dir in (joinpath(ROOT, "src"), joinpath(ROOT, "test"))
        for (root, _, fs) in walkdir(dir), f in fs
            endswith(f, ".jl") && push!(files, joinpath(root, f))
        end
    end
    # the extension entry files, not the per-problem directories
    append!(files, [joinpath(EXT, f) for f in readdir(EXT) if endswith(f, ".jl")])
    return files
end

"""No list of problem names written by hand in `src/`, `test/`, or the extension files."""
function test_no_handwritten_list()
    Test.@testset "no hand-written list" verbose = VERBOSE begin
        problems = OCP.problems(; status=:all)
        pattern = Regex("(?<![:\\w]):(" * join(string.(problems), "|") * ")\\b|\"(" *
                        join(string.(problems), "|") * ")\"")
        files = source_files()
        Test.@test !isempty(files)
        for file in files
            in_docstring = false
            for (i, line) in enumerate(eachline(file))
                # docstrings may show a problem as an example; code may not name one
                isodd(count("\"\"\"", line)) && (in_docstring = !in_docstring; continue)
                in_docstring && continue
                m = match(pattern, line)
                isnothing(m) && continue
                @error "Problem name written by hand" file = relpath(file, ROOT) line = i text = strip(line)
                Test.@test isnothing(m)
            end
        end
    end
end

function test_consistency()
    Test.@testset "consistency of the collection" verbose = VERBOSE showtiming = SHOWTIMING begin
        test_files()
        test_schema()
        test_comparison_rule()
        test_reference_objectives()
        test_no_handwritten_list()
    end
end

end # module TestConsistency

test_consistency() = TestConsistency.test_consistency()
