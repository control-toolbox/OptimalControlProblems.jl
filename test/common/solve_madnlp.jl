# Solve an NLP model with MadNLP (MUMPS) and the options of `Main.TestConfigs`.
module SolveMadNLP

using MadNLP: MadNLP
using MadNLPMumps: MadNLPMumps

function solve(nlp; max_iter::Int=Main.TestConfigs.MAX_ITER)
    o = Main.TestConfigs
    return MadNLP.madnlp(
        nlp;
        print_level=MadNLP.ERROR,
        tol=o.TOL,
        max_iter=max_iter,
        max_wall_time=o.MAX_WALL_TIME,
        linear_solver=MadNLPMumps.MumpsSolver,
    )
end

succeeded(stats) = stats.status == MadNLP.SOLVE_SUCCEEDED

end # module SolveMadNLP
