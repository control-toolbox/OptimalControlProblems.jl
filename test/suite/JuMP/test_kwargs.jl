module TestKwargsJuMP

using Test: Test
using JuMP: JuMP
using Ipopt: Ipopt
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

function test_kwargs()
    Test.@testset "JuMP keyword arguments" verbose = VERBOSE showtiming = SHOWTIMING begin
        Main.TestStatus.for_each_problem("kwargs", OptimalControlProblems.JuMPBackend()) do problem
            build = getfield(OptimalControlProblems, problem)
            # arguments of the model constructor are forwarded to JuMP.Model
            model = build(OptimalControlProblems.JuMPBackend(), Ipopt.Optimizer; add_bridges=true)
            Test.@test JuMP.solver_name(model) == "Ipopt"
        end
    end
end

end # module TestKwargsJuMP

test_kwargs() = TestKwargsJuMP.test_kwargs()
