isdefined(Main, :Comparison) || include(joinpath(@__DIR__, "..", "..", "common", "comparison.jl"))

module TestQuickComparison

using Test: Test
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# Small grid: the same discretization is used by every modeling, so objectives must agree.
const GRID_SIZE = 20

function test_quick()
    C = Main.Comparison
    Test.@testset "objectives of the modelings (small grid)" verbose = VERBOSE showtiming = SHOWTIMING begin
        for problem in C.problems()
            Test.@testset "$problem" begin
                try
                    grid_size = GRID_SIZE
                    data = C.solve_all(problem; grid_size=grid_size, max_iter=Main.TestSolverOptions.MAX_ITER)
                    for other in (:OptimalControl, :OptimalControlScalarized)
                        Test.@testset "$(C.REFERENCE) vs $other" begin
                            C.compare_objective(data[C.REFERENCE], data[other])
                        end
                    end
                catch err
                    Main.TestHelpers.broken_if_domain_error(err)
                end
            end
        end
    end
end

end # module TestQuickComparison

test_quick() = TestQuickComparison.test_quick()
