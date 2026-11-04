isdefined(Main, :Comparison) || include(joinpath(@__DIR__, "..", "..", "common", "comparison.jl"))

module TestInitComparison

using Test: Test
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# Small grid: the structure and the initial guess are compared at every grid point, the
# default grid is covered by cron/comparison.
const GRID_SIZE = 20

function test_init()
    C = Main.Comparison
    Test.@testset "comparison at the initial guess (0 iterations)" verbose = VERBOSE showtiming = SHOWTIMING begin
        for problem in C.problems()
            Test.@testset "$problem" begin
                try
                    grid_size = GRID_SIZE
                    data = C.solve_all(problem; grid_size=grid_size, max_iter=0)
                    for other in (:OptimalControl, :OptimalControlScalarized)
                        Test.@testset "$(C.REFERENCE) vs $other" begin
                            ref, cmp = data[C.REFERENCE], data[other]
                            C.compare_structure(ref, cmp)
                            if C.compare_grid(ref, cmp)
                                C.compare_trajectories(ref, cmp)
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

end # module TestInitComparison

test_init() = TestInitComparison.test_init()
