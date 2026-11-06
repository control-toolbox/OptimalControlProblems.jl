isdefined(Main, :Comparison) ||
    include(joinpath(@__DIR__, "..", "..", "common", "comparison.jl"))

module TestInitComparison

using Test: Test
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# Small grid: the structure and the initial guess are compared at every grid point; the
# default grid is covered by cron/comparison.

function test_init()
    C = Main.Comparison
    Test.@testset "comparison at the initial guess (0 iterations)" verbose = VERBOSE showtiming =
        SHOWTIMING begin
        C.for_each_comparison(
            "test_init"; grid_size=Main.TestConfigs.SMALL_GRID, max_iter=0
        ) do problem, ref, cmp
            C.compare_structure(ref, cmp)
            if C.compare_grid(ref, cmp)
                C.compare_trajectories(ref, cmp)
            end
            C.compare_objective(ref, cmp)
        end
    end
end

end # module TestInitComparison

test_init() = TestInitComparison.test_init()
