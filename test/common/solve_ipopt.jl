# Solve an NLP model with Ipopt and the options of `Main.TestConfigs`.
module SolveIpopt

using NLPModelsIpopt: NLPModelsIpopt

function solve(nlp; max_iter::Int=Main.TestConfigs.MAX_ITER)
    o = Main.TestConfigs
    return NLPModelsIpopt.ipopt(
        nlp;
        print_level=0,
        tol=o.TOL,
        mu_strategy=o.MU_STRATEGY,
        sb=o.SB,
        max_iter=max_iter,
        max_wall_time=o.MAX_WALL_TIME,
    )
end

succeeded(stats) = stats.status in (:first_order, :acceptable)

end # module SolveIpopt
