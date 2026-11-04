isdefined(Main, :Comparison) || include(joinpath(@__DIR__, "..", "..", "common", "comparison.jl"))

module TestQuickComparison

using Test: Test
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# Same discretization in every modeling, so the objectives must agree.

function test_quick()
    C = Main.Comparison
    Test.@testset "objectives of the modelings (small grid)" verbose = VERBOSE showtiming = SHOWTIMING begin
        C.for_each_comparison("test_quick"; grid_size=Main.TestConfigs.SMALL_GRID, max_iter=Main.TestConfigs.MAX_ITER) do problem, ref, cmp
            C.compare_objective(ref, cmp)
        end
    end
end

end # module TestQuickComparison

test_quick() = TestQuickComparison.test_quick()
