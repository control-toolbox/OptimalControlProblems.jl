module TestStartComparison

using Test: Test
using JuMP: JuMP
using OptimalControl: OptimalControl
using OptimalControlProblems: OptimalControlProblems

const OCP = OptimalControlProblems
const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

const SCHEMES = (:trapeze, :euler, :euler_implicit, :midpoint)
const GRID_SIZE = 4
const TOL = 1e-12

# The initial guess is shared by every modeling (#377): the start values of the JuMP model
# are the initial point of the NLP of CTDirect, for every scheme. The interleaved JuMP model
# has CTDirect's order of variables (#370).
function test_start()
    jump, oc = OCP.JuMPBackend(), OCP.OptimalControlBackend()
    Test.@testset "start values: JuMP vs OptimalControl (CTDirect)" verbose = VERBOSE showtiming =
        SHOWTIMING begin
        Main.TestStatus.for_each_problem("test_start", jump, oc) do problem
            build = getfield(OCP, problem)
            pb = build(oc)
            ocp = OCP.model(pb)
            init = OptimalControl.build_initial_guess(ocp, OCP.initial_guess(pb))
            for scheme in SCHEMES
                model = build(jump; grid_size=GRID_SIZE, scheme=scheme)
                start = JuMP.start_value.(JuMP.all_variables(model))
                docp = OptimalControl.discretize(
                    ocp, OptimalControl.Collocation(; grid_size=GRID_SIZE, scheme=scheme)
                )
                x0 = OptimalControl.nlp_model(docp, init, OptimalControl.ADNLP()).meta.x0
                Test.@test length(start) == length(x0)
                Test.@test all(@. abs(start - x0) <= TOL * max(1, abs(x0)))
            end
        end
    end
end

end # module TestStartComparison

test_start() = TestStartComparison.test_start()
