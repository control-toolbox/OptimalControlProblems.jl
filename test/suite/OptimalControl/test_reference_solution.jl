module TestReferenceSolutionOptimalControl

using Test: Test
using NLPModels: NLPModels
using OptimalControl: OptimalControl # loads the OptimalControlModels extension
using OptimalControlProblems: OptimalControlProblems

const OCP = OptimalControlProblems
const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

const GRID_SIZES = (100, 400)

# Constraint violation of the discretized problem at the analytic solution, and relative gap
# of its objective to the analytic objective, on `grid_size` steps of the trapezoidal rule.
function errors(ocp, solution, grid_size)
    init = OptimalControl.build_initial_guess(ocp, (state=solution.state, control=solution.control))
    docp = OptimalControl.discretize(ocp, OptimalControl.Collocation(; grid_size=grid_size, scheme=:trapeze))
    nlp = OptimalControl.nlp_model(docp, init, OptimalControl.ADNLP())
    x, meta = nlp.meta.x0, nlp.meta
    c = NLPModels.cons(nlp, x)
    violation = max(
        maximum(max.(meta.lcon .- c, c .- meta.ucon, 0); init=0.0),
        maximum(max.(meta.lvar .- x, x .- meta.uvar, 0); init=0.0),
    )
    gap = abs(NLPModels.obj(nlp, x) - solution.objective) / abs(solution.objective)
    return violation, gap
end

# The analytic solution (#379) solves the model: at the solution, the constraint violation of
# the discretized problem and the gap of its objective to the analytic one vanish with the
# step, at the order of the trapezoidal rule (2; 4 steps more divide them by 16). A wrong
# solution or objective leaves them bounded away from zero.
function test_reference_solution()
    backend = OCP.OptimalControlBackend()
    Test.@testset "OptimalControl reference solution" verbose = VERBOSE showtiming = SHOWTIMING begin
        Main.TestStatus.for_each_problem("reference_solution", backend) do problem
            solution = OCP.reference_solution_data(problem)
            isnothing(solution) && return nothing
            ocp = OCP.model(getfield(OCP, problem)(backend))
            (v1, g1), (v2, g2) = (errors(ocp, solution, N) for N in GRID_SIZES)
            Test.@test v2 <= v1 / 10 && v2 < 1e-4
            Test.@test g2 <= g1 / 10 && g2 < 1e-4
        end
    end
end

end # module TestReferenceSolutionOptimalControl

test_reference_solution() = TestReferenceSolutionOptimalControl.test_reference_solution()
