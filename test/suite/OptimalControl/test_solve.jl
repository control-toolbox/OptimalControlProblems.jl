module TestSolveOptimalControl

using Test: Test
using CTDirect: CTDirect
using NLPModelsIpopt: NLPModelsIpopt
using OptimalControl: OptimalControl # loads the OptimalControlModels extension
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# Small grid: the full convergence study on the default grid is in cron/.
const GRID_SIZE = 20

function test_solve()
    options = Main.TestSolverOptions
    ipopt_options = Dict(
        :print_level => 0,
        :tol => options.TOL,
        :mu_strategy => options.MU_STRATEGY,
        :sb => options.SB,
        :max_iter => options.MAX_ITER,
        :max_wall_time => options.MAX_WALL_TIME,
    )
    Test.@testset "OptimalControl solve (small grid)" verbose = VERBOSE showtiming = SHOWTIMING begin
        for problem in OptimalControlProblems.problems(OptimalControlProblems.OptimalControlBackend())
            Test.@testset "$problem" begin
                try
                    build = getfield(OptimalControlProblems, problem)
                    docp = build(
                        OptimalControlProblems.OptimalControlBackend(); grid_size=GRID_SIZE
                    )
                    nlp = CTDirect.nlp_model(docp)
                    sol = NLPModelsIpopt.ipopt(nlp; ipopt_options...)
                    Main.TestHelpers.@broken_if_false sol.status in (:first_order, :acceptable)
                catch err
                    Main.TestHelpers.broken_if_domain_error(err)
                end
            end
        end
    end
end

end # module TestSolveOptimalControl

test_solve() = TestSolveOptimalControl.test_solve()
