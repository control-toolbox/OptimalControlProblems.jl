module TestModelings

using Test: Test
using CTBase: CTBase
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# Deliberately no problem name written here: everything is derived from the catalogue.
const BACKENDS = (
    OptimalControlProblems.JuMPBackend(),
    OptimalControlProblems.OptimalControlBackend(),
    OptimalControlProblems.OptimalControlScalarizedBackend(),
)

function test_modelings()
    Test.@testset "modeling lists" verbose = VERBOSE showtiming = SHOWTIMING begin
        all_problems = OptimalControlProblems.problems(; status=:all)
        Test.@test issorted(all_problems)
        Test.@test !isempty(OptimalControlProblems.problems())
        for backend in BACKENDS
            listed = OptimalControlProblems.problems(backend; status=:all)
            Test.@test listed ⊆ all_problems
            for status in (:valid, :draft)
                Test.@test OptimalControlProblems.problems(backend; status=status) ⊆ listed
            end
            Test.@test sort!(
                vcat(
                    OptimalControlProblems.problems(backend; status=:valid),
                    OptimalControlProblems.problems(backend; status=:draft),
                ),
            ) == listed
        end
        for a in BACKENDS, b in BACKENDS
            Test.@test OptimalControlProblems.problems(a, b) ==
                intersect(OptimalControlProblems.problems(a), OptimalControlProblems.problems(b))
        end
        Test.@test_throws CTBase.IncorrectArgument OptimalControlProblems.problems(;
            status=:unknown
        )
    end

    Test.@testset "status" verbose = VERBOSE showtiming = SHOWTIMING begin
        Test.@test_throws CTBase.IncorrectArgument OptimalControlProblems.status(
            :not_a_problem
        )
    end

    Test.@testset "missing modeling" verbose = VERBOSE showtiming = SHOWTIMING begin
        all_problems = OptimalControlProblems.problems(; status=:all)
        for backend in BACKENDS
            missing_problems = setdiff(
                all_problems, OptimalControlProblems.problems(backend; status=:all)
            )
            isempty(missing_problems) && continue
            build = getfield(OptimalControlProblems, first(missing_problems))
            err = try
                build(backend)
                nothing
            catch exception
                exception
            end
            Test.@test err isa CTBase.IncorrectArgument
            Test.@test occursin("problems(", sprint(showerror, err))
        end
    end

    Test.@testset "generated documentation" verbose = VERBOSE showtiming = SHOWTIMING begin
        for problem in OptimalControlProblems.problems(; status=:all)
            documentation = string(Base.Docs.doc(getfield(OptimalControlProblems, problem)))
            Test.@test !occursin("TYPEDSIGNATURES", documentation)
            Test.@test !occursin("TypedMethodSignatures", documentation)
        end
    end
end

end # module TestModelings

test_modelings() = TestModelings.test_modelings()
