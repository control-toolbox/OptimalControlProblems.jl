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

"""
The options of the solver `name` for a solve: the defaults of `Main.TestConfigs`, replaced by
`options` (robustness, #378). As given to the solver, before `barrier`; the results of the
solves record them (#409).
"""
function solver_options(name::Symbol; max_iter::Int, options::NamedTuple=(;))
    o = Main.TestConfigs
    defaults = if name === :ipopt
        (
            print_level=0,
            tol=o.TOL,
            mu_strategy=o.MU_STRATEGY,
            sb=o.SB,
            max_iter=max_iter,
            max_wall_time=o.MAX_WALL_TIME,
        )
    elseif name in (:madnlp, :madnlp_gpu)
        (print_level=MadNLP.ERROR, tol=o.TOL, max_iter=max_iter, max_wall_time=o.MAX_WALL_TIME)
    else
        error("unknown solver $(name) in the test configurations")
    end
    return merge(defaults, options)
end

function solver(name::Symbol; max_iter::Int, options::NamedTuple=(;))
    all = solver_options(name; max_iter=max_iter, options=options)
    name === :ipopt && return OptimalControl.Ipopt(; all...)
    T = name === :madnlp ? OptimalControl.MadNLP : OptimalControl.MadNLP{OptimalControl.GPU}
    return T(; barrier(all)...)
end

"""
How a solve with `config` and `options` was configured, as `TestStatus.result!` records it
(#409): the modeler, the solver and every option of the solver.
"""
function settings(config; max_iter::Int=Main.TestConfigs.MAX_ITER, options::NamedTuple=(;))
    return (
        modeler=config.modeler,
        solver=config.solver,
        options=solver_options(config.solver; max_iter=max_iter, options=options),
    )
end

function discretizer(grid_size::Int)
    return OptimalControl.Collocation(; grid_size=grid_size, scheme=Main.TestConfigs.SCHEME)
end

"""Solve the `Problem` `pb` with `config`; the default grid of the problem if `grid_size` is `nothing`."""
function solve(
    pb, config; grid_size::Int, max_iter::Int=Main.TestConfigs.MAX_ITER, options::NamedTuple=(;)
)
    return OptimalControl.solve(
        OptimalControlProblems.model(pb);
        init=OptimalControlProblems.initial_guess(pb),
        discretizer=discretizer(grid_size),
        modeler=MODELERS[config.modeler](),
        solver=solver(config.solver; max_iter=max_iter, options=options),
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

# MadNLP 0.10 reads the barrier parameter from its `barrier` option only: a top-level
# `mu_init` is ignored (with a warning hidden by `print_level=ERROR`), and the MadNLP strategy
# of OptimalControl knows `mu_init` but not `barrier`. So `mu_init` and `barrier=:loqo` become
# a `barrier`, passed with `bypass`: the other options are still validated.
function barrier(options::NamedTuple)
    haskey(options, :mu_init) || get(options, :barrier, nothing) === :loqo || return options
    update = if haskey(options, :mu_init)
        MadNLP.MonotoneUpdate(; mu_init=options.mu_init)
    else
        MadNLP.LOQOUpdate()
    end
    rest = Base.structdiff(options, NamedTuple{(:mu_init, :barrier)})
    return merge(rest, (barrier=OptimalControl.bypass(update),))
end

succeeded(sol) = OptimalControl.successful(sol)

"""What `TestStatus.result!` records of a solution."""
function outcome(sol)
    return (
        converged=OptimalControl.successful(sol),
        reason=OptimalControl.status(sol),
        objective=OptimalControl.objective(sol),
        iterations=OptimalControl.iterations(sol),
        violation=OptimalControl.constraints_violation(sol),
    )
end

end # module SolveOptimalControl
