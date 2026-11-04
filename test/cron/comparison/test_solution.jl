isdefined(Main, :Comparison) || include(joinpath(@__DIR__, "..", "..", "common", "comparison.jl"))

module TestSolutionComparison

using Test: Test
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

function test_solution()
    C = Main.Comparison
    Test.@testset "solutions of the modelings (default grid)" verbose = VERBOSE showtiming = SHOWTIMING begin
        for problem in C.problems()
            Test.@testset "$problem" begin
                try
                    grid_size = OptimalControlProblems.metadata(problem)[:grid_size]
                    data = C.solve_all(problem; grid_size=grid_size, max_iter=Main.TestSolverOptions.MAX_ITER)
                    for other in (:OptimalControl, :OptimalControlScalarized)
                        Test.@testset "$(C.REFERENCE) vs $other" begin
                            ref, cmp = data[C.REFERENCE], data[other]
                            C.compare_structure(ref, cmp)
                            if C.compare_grid(ref, cmp)
                                # the control of `jackson` is not unique
                                C.compare_trajectories(ref, cmp; check_control=problem != :jackson)
                            end
                            C.compare_objective(ref, cmp)
                        end
                    end
                catch err
                    Main.TestHelpers.broken_if_domain_error(err)
                end
            end
        end
    end
end

end # module TestSolutionComparison

test_solution() = TestSolutionComparison.test_solution()
