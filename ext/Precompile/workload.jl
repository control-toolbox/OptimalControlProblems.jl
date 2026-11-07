# The precompilation workload (#373, prototype): solve a few problems of the OptimalControl
# modeling with a given modeler and solver, so that their compiled code is stored in the
# package image of the extension. Almost all of it is specific to each problem (the measures
# of #373), so a problem is precompiled only if it is solved here.
const PRECOMPILED = (:vanderpol, :ducted_fan, :insurance)
const GRID_SIZE = 10

function workload(modeler, solver)
    OCP = OptimalControlProblems
    for problem in PRECOMPILED
        pb = getfield(OCP, problem)(OCP.OptimalControlBackend())
        OptimalControl.solve(
            OCP.model(pb);
            init=OCP.initial_guess(pb),
            discretizer=OptimalControl.Collocation(; grid_size=GRID_SIZE, scheme=:trapeze),
            modeler=modeler,
            solver=solver,
            display=false,
        )
    end
    return nothing
end
