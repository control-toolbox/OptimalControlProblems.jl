isdefined(Main, :SolveModelingToolkit) ||
    include(joinpath(@__DIR__, "..", "..", "common", "solve_modelingtoolkit.jl"))

module TestReferenceSolutionModelingToolkit

using Test: Test
using ModelingToolkit: ModelingToolkit as MTK
using OptimalControlProblems: OptimalControlProblems as OCP

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

function test_reference_solution()
    backend = OCP.ModelingToolkitBackend()
    Test.@testset "ModelingToolkit reference solution" verbose = VERBOSE showtiming = SHOWTIMING begin
        Main.TestStatus.for_each_problem("reference_solution", backend) do problem
            ref = OCP.reference_solution_data(problem)
            isnothing(ref) && return nothing
            pb = getfield(OCP, problem)(backend)
            
            # Solve with Corleone + Ipopt
            res = Main.SolveModelingToolkit.solve(pb; grid_size=30, max_iter=100)
            Test.@test Main.SolveModelingToolkit.succeeded(res)
            # The numerical objective converges towards the reference objective (4.5 for bryson_denham)
            rel_err = abs(res.objective - ref.objective) / abs(ref.objective)
            Test.@test rel_err < 0.05
        end
    end
end

end # module TestReferenceSolutionModelingToolkit

test_reference_solution() = TestReferenceSolutionModelingToolkit.test_reference_solution()
