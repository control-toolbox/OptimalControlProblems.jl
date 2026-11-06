module TestFeaturesOptimalControl

using Test: Test
using OptimalControl: OptimalControl # loads the OptimalControlModels extension
using OptimalControlProblems: OptimalControlProblems

const OCP = OptimalControlProblems
const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# The features (#376) are computed from the model: they agree with it, and with the
# functions of OptimalControl that give the same information.
function test_features()
    backend = OCP.OptimalControlBackend()
    Test.@testset "OptimalControl features" verbose = VERBOSE showtiming = SHOWTIMING begin
        Main.TestStatus.for_each_problem("features", backend) do problem
            pb = getfield(OCP, problem)(backend)
            ocp = OCP.model(pb)
            f = OCP.features(problem)
            Test.@test f == OCP.features(pb)
            Test.@test f.state_dimension == OptimalControl.state_dimension(ocp)
            Test.@test f.control_dimension == OptimalControl.control_dimension(ocp)
            Test.@test f.variable_dimension == OptimalControl.variable_dimension(ocp)
            Test.@test f.cost in (:mayer, :lagrange, :bolza)
            Test.@test (f.cost in (:mayer, :bolza)) == OptimalControl.has_mayer_cost(ocp)
            Test.@test (f.cost in (:lagrange, :bolza)) == OptimalControl.has_lagrange_cost(ocp)
            Test.@test f.initial_time in (:fixed, :free) && f.final_time in (:fixed, :free)
            # a free time is a component of the variable
            Test.@test f.variable_dimension > 0 || (f.initial_time === :fixed && f.final_time === :fixed)
            Test.@test keys(f.constraints) == (:state_box, :control_box, :variable_box, :path, :boundary)
            Test.@test all(n -> n isa Int && n >= 0, f.constraints)
        end
    end
end

end # module TestFeaturesOptimalControl

test_features() = TestFeaturesOptimalControl.test_features()
