module TestSolveOptimalControlScalarized

using Test: Test
using CTDirect: CTDirect
using MadNLP: MadNLP
using MadNLPMumps: MadNLPMumps
using OptimalControl: OptimalControl # loads the OptimalControlModels extension
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# Small grid: the full convergence study on the default grid is in cron/.
const GRID_SIZE = 20

function test_solve()
    options = Main.TestSolverOptions
    madnlp_options = Dict(
        :print_level => MadNLP.ERROR,
        :tol => options.TOL,
        :max_iter => options.MAX_ITER,
        :max_wall_time => options.MAX_WALL_TIME,
        :linear_solver => MadNLPMumps.MumpsSolver,
    )
    Test.@testset "OptimalControlScalarized solve (small grid)" verbose = VERBOSE showtiming = SHOWTIMING begin
        for problem in OptimalControlProblems.problems()
            Test.@testset "$problem" begin
                try
                    # transitional: the scalarized variants are the `_s` functions until #303
                    build = getfield(OptimalControlProblems, Symbol(problem, :_s))
                    docp = build(
                        OptimalControlProblems.OptimalControlBackend(),
                        :madnlp,
                        :exa;
                        grid_size=GRID_SIZE,
                    )
                    nlp = CTDirect.nlp_model(docp)
                    sol = MadNLP.madnlp(nlp; madnlp_options...)
                    Main.TestHelpers.@broken_if_false sol.status == MadNLP.SOLVE_SUCCEEDED
                catch err
                    Main.TestHelpers.broken_if_domain_error(err)
                end
            end
        end
    end
end

end # module TestSolveOptimalControlScalarized

test_solve() = TestSolveOptimalControlScalarized.test_solve()
