module TestConvergenceOptimalControl

using Test: Test
using CTDirect: CTDirect
using NLPModelsIpopt: NLPModelsIpopt
using OptimalControl: OptimalControl # loads the OptimalControlModels extension
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# Default grid of each problem, two solves in a row, as the former test_OptimalControl.

function test_convergence()
    options = Main.TestSolverOptions
    ipopt_options = Dict(
        :print_level => 0,
        :tol => options.TOL,
        :mu_strategy => options.MU_STRATEGY,
        :sb => options.SB,
        :max_iter => options.MAX_ITER,
        :max_wall_time => options.MAX_WALL_TIME,
    )
    Test.@testset "OptimalControl convergence (default grid)" verbose = VERBOSE showtiming = SHOWTIMING begin
        for problem in OptimalControlProblems.problems(OptimalControlProblems.OptimalControlBackend())
            Test.@testset "$problem" begin
                try
                    build = getfield(OptimalControlProblems, problem)
                    docp = build(OptimalControlProblems.OptimalControlBackend())
                    nlp = CTDirect.nlp_model(docp)
                    sol = NLPModelsIpopt.ipopt(nlp; ipopt_options...)
                    sol = NLPModelsIpopt.ipopt(nlp; ipopt_options...) # second solve
                    Main.TestHelpers.@broken_if_false sol.status in (:first_order, :acceptable)
                catch err
                    Main.TestHelpers.broken_if_domain_error(err)
                end
            end
        end
    end
end

end # module TestConvergenceOptimalControl

test_convergence() = TestConvergenceOptimalControl.test_convergence()
