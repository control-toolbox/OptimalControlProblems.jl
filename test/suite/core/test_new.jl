module TestNew

using Test: Test
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

const TOOLS = joinpath(pkgdir(OptimalControlProblems), "tools")
include(joinpath(TOOLS, "new", "NewProblem.jl"))
include(joinpath(TOOLS, "lint", "ProblemLint.jl"))
const N = NewProblem
const L = ProblemLint

failures(problem, root) = [
    r.message for r in L.checks(L.source(problem; root)) if r.status === :fail
]

# what the person writes in place of each `TODO`
function fill_todos(root)
    for (dir, _, files) in walkdir(joinpath(root, "ext")), file in files
        path = joinpath(dir, file)
        write(path, replace(read(path, String), "TODO" => "Written"))
    end
end

statuses(problem, root) = L.metadata(problem; root)[:status]

function test_new()
    Test.@testset "new problem" verbose = VERBOSE showtiming = SHOWTIMING begin
        Test.@testset "a template per modeling, nothing else" begin
            Test.@test sort(readdir(N.TEMPLATES)) == sort([
                "metadata.jl", "description.md", ("$(m).jl" for m in N.modelings())...
            ])
        end

        Test.@testset "every modeling: lint fails on the TODOs only" begin
            mktempdir() do root
                paths = N.create(:foo, N.modelings(); root)
                Test.@test length(paths) == 2 + length(N.modelings())
                for path in paths
                    endswith(path, ".jl") || continue
                    Test.@test Meta.parseall(read(path, String)) isa Expr
                end
                Test.@test keys(statuses(:foo, root)) == Tuple(N.modelings())
                Test.@test all(==(:draft), values(statuses(:foo, root)))
                failed = failures(:foo, root)
                Test.@test length(failed) == 1
                Test.@test startswith(only(failed), "nothing left to fill (`TODO`)")
                fill_todos(root)
                Test.@test isempty(failures(:foo, root))
            end
        end

        Test.@testset "some modelings, then add one" begin
            mktempdir() do root
                N.create(:foo, [:JuMP, :OptimalControl]; root)
                Test.@test keys(statuses(:foo, root)) == (:OptimalControl, :JuMP)
                fill_todos(root)
                Test.@test isempty(failures(:foo, root))
                # a status set by hand is kept, the new modeling comes in its place
                path = N.metadata_path(root, :foo)
                write(path, replace(read(path, String), "JuMP=:draft" => "JuMP=:valid"))
                N.add(:foo, [:ModelingToolkit]; root)
                Test.@test statuses(:foo, root) ==
                    (OptimalControl=:draft, JuMP=:valid, ModelingToolkit=:draft)
                failed = failures(:foo, root)
                Test.@test failed == ["nothing left to fill (`TODO`): model ModelingToolkit"]
            end
        end

        Test.@testset "refusals" begin
            mktempdir() do root
                Test.@test_throws ArgumentError N.create(:foo, Symbol[]; root)
                Test.@test_throws ArgumentError N.create(:foo, [:Fortran]; root)
                Test.@test_throws ArgumentError N.create(:foo, [:JuMP, :JuMP]; root)
                Test.@test_throws ArgumentError N.create(Symbol("foo bar"), [:JuMP]; root)
                Test.@test_throws ArgumentError N.add(:foo, [:JuMP]; root)
                N.create(:foo, [:JuMP]; root)
                Test.@test_throws ArgumentError N.create(:foo, [:OptimalControl]; root)
                Test.@test_throws ArgumentError N.add(:foo, [:JuMP]; root)
                # nothing written by a refusal
                Test.@test !isfile(N.model_path(root, :foo, :OptimalControl))
            end
        end
    end
end

end # module TestNew

test_new() = TestNew.test_new()
