module TestConvergenceOptimalControlScalarized

using Test: Test
using CTDirect: CTDirect
using MadNLP: MadNLP
using MadNLPMumps: MadNLPMumps
using OptimalControl: OptimalControl # loads the OptimalControlModels extension
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# Default grid of each problem, two solves in a row, as the former test_OptimalControl_s.

function test_convergence()
    options = Main.TestSolverOptions
    madnlp_options = Dict(
        :print_level => MadNLP.ERROR,
        :tol => options.TOL,
        :max_iter => options.MAX_ITER,
        :max_wall_time => options.MAX_WALL_TIME,
        :linear_solver => MadNLPMumps.MumpsSolver,
    )
    Test.@testset "OptimalControlScalarized convergence (default grid)" verbose = VERBOSE showtiming =
        SHOWTIMING begin
        for problem in OptimalControlProblems.problems(
            OptimalControlProblems.OptimalControlScalarizedBackend()
        )
            Test.@testset "$problem" begin
                try
                    # transitional: the scalarized variants are the `_s` functions until #303
                    build = getfield(OptimalControlProblems, problem)
                    docp = build(
                        OptimalControlProblems.OptimalControlScalarizedBackend(),
                        :madnlp,
                        :exa,
                    )
                    nlp = CTDirect.nlp_model(docp)
                    sol = MadNLP.madnlp(nlp; madnlp_options...)
                    sol = MadNLP.madnlp(nlp; madnlp_options...) # second solve
                    Main.TestHelpers.@broken_if_false sol.status == MadNLP.SOLVE_SUCCEEDED
                catch err
                    Main.TestHelpers.broken_if_domain_error(err)
                end
            end
        end
    end
end

end # module TestConvergenceOptimalControlScalarized

test_convergence() = TestConvergenceOptimalControlScalarized.test_convergence()
