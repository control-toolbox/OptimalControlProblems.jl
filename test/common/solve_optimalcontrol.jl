# Solve a problem of an OptimalControl modeling with OptimalControl v2, in explicit mode:
# the configuration of `Main.TestConfigs` names the modeler and the solver, the
# discretization is the trapezoidal rule (the schemes are #284).
module SolveOptimalControl

using MadNLP: MadNLP
using NLPModelsIpopt: NLPModelsIpopt # loads the Ipopt strategy of OptimalControl
using OptimalControl: OptimalControl
using OptimalControlProblems: OptimalControlProblems

# The GPU strategies need CUDA (modeler) and MadNLPGPU, CUDA, CUDSS (solver) loaded: the
# environment `GPU` loads them.
const MODELERS = Dict(
    :adnlp => OptimalControl.ADNLP,
    :exa => OptimalControl.Exa,
    :exa_gpu => OptimalControl.Exa{OptimalControl.GPU},
)

function solver(name::Symbol; max_iter::Int)
    o = Main.TestConfigs
    if name === :ipopt
        return OptimalControl.Ipopt(;
            print_level=0,
            tol=o.TOL,
            mu_strategy=o.MU_STRATEGY,
            sb=o.SB,
            max_iter=max_iter,
            max_wall_time=o.MAX_WALL_TIME,
        )
    elseif name in (:madnlp, :madnlp_gpu)
        T = name === :madnlp ? OptimalControl.MadNLP : OptimalControl.MadNLP{OptimalControl.GPU}
        return T(;
            print_level=MadNLP.ERROR,
            tol=o.TOL,
            max_iter=max_iter,
            max_wall_time=o.MAX_WALL_TIME,
        )
    end
    return error("unknown solver $(name) in the test configurations")
end

function discretizer(grid_size::Int)
    return OptimalControl.Collocation(; grid_size=grid_size, scheme=Main.TestConfigs.SCHEME)
end

"""Solve the `Problem` `pb` with `config`; the default grid of the problem if `grid_size` is `nothing`."""
function solve(pb, config; grid_size::Int, max_iter::Int=Main.TestConfigs.MAX_ITER)
    return OptimalControl.solve(
        OptimalControlProblems.model(pb);
        init=OptimalControlProblems.initial_guess(pb),
        discretizer=discretizer(grid_size),
        modeler=MODELERS[config.modeler](),
        solver=solver(config.solver; max_iter=max_iter),
        display=false,
    )
end

"""NLP model of the `Problem` `pb` with `config`, built as `solve` builds it."""
function nlp(pb, config; grid_size::Int)
    ocp = OptimalControlProblems.model(pb)
    docp = OptimalControl.discretize(ocp, discretizer(grid_size))
    init = OptimalControl.build_initial_guess(ocp, OptimalControlProblems.initial_guess(pb))
    return OptimalControl.nlp_model(docp, init, MODELERS[config.modeler]())
end

succeeded(sol) = OptimalControl.successful(sol)

"""What `TestStatus.result!` records of a solution."""
function outcome(sol)
    return (
        converged=OptimalControl.successful(sol),
        reason=OptimalControl.status(sol),
        objective=OptimalControl.objective(sol),
        iterations=OptimalControl.iterations(sol),
    )
end

end # module SolveOptimalControl
