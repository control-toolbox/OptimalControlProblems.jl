isdefined(Main, :Comparison) || include(joinpath(@__DIR__, "..", "..", "common", "comparison.jl"))

module TestSolutionComparison

using Test: Test
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

default_grid(problem) = OptimalControlProblems.metadata(problem)[:grid_size]

function test_solution()
    C = Main.Comparison
    Test.@testset "solutions of the modelings (default grid)" verbose = VERBOSE showtiming = SHOWTIMING begin
        C.for_each_comparison("test_solution"; grid_size=default_grid, max_iter=Main.TestConfigs.MAX_ITER) do problem, ref, cmp
            C.compare_structure(ref, cmp)
            if C.compare_grid(ref, cmp)
                # the control of a problem may not be unique (see `:notes` of its metadata)
                C.compare_trajectories(ref, cmp; check_control=C.unique_control(problem))
            end
            C.compare_objective(ref, cmp)
        end
    end
end

end # module TestSolutionComparison

test_solution() = TestSolutionComparison.test_solution()
