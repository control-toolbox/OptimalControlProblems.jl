# Time and compilation time of each stage of a solve, in a fresh session (#373): building the
# problem, discretizing, the NLP model (ADNLP, Exa), its first evaluation, two solves (ADNLP
# + Ipopt, Exa + MadNLP; 3 iterations, grid 10). The first problem of a session also pays
# what is shared; run the problems in two orders to tell them apart.
#
# Usage, in an environment with MadNLP, NLPModels, NLPModelsIpopt, OptimalControl and this
# package (developed):
#
#     julia --project=<env> test/extras/compile_time.jl vanderpol ducted_fan insurance
t_load = @timed begin
    using MadNLP: MadNLP
    using NLPModelsIpopt: NLPModelsIpopt
    using NLPModels: NLPModels
    using OptimalControl: OptimalControl
    using OptimalControlProblems: OptimalControlProblems
end
const OCP = OptimalControlProblems
const OC = OptimalControl
const GRID = 10
function row(p, cfg, stage, t)
    return println(
        join(
            (p, cfg, stage, round(t.time; digits=2), round(t.compile_time; digits=2)), '\t'
        ),
    )
end
println(join(("problem", "config", "stage", "time_s", "compile_s"), '\t'))
row(:none, :none, :load, t_load)
function solver(s)
    return if s === :ipopt
        OC.Ipopt(; print_level=0, max_iter=3, sb="yes")
    else
        OC.MadNLP(; print_level=MadNLP.ERROR, max_iter=3)
    end
end
const MODELERS = Dict(:adnlp => OC.ADNLP, :exa => OC.Exa)
for p in Symbol.(ARGS)
    t = @timed getfield(OCP, p)(OCP.OptimalControlBackend())
    row(p, :all, :build, t)
    pb = t.value
    ocp = OCP.model(pb)
    t = @timed OC.discretize(ocp, OC.Collocation(; grid_size=GRID, scheme=:trapeze))
    row(p, :all, :discretize, t)
    docp = t.value
    init = OC.build_initial_guess(ocp, OCP.initial_guess(pb))
    for (m, s) in ((:adnlp, :ipopt), (:exa, :madnlp))
        cfg = Symbol(m, "+", s)
        t = @timed OC.nlp_model(docp, init, MODELERS[m]())
        row(p, cfg, :nlp_model, t)
        nlp = t.value
        x = nlp.meta.x0
        y = ones(nlp.meta.ncon)
        t = @timed (NLPModels.obj(nlp, x);
        NLPModels.grad(nlp, x);
        NLPModels.cons(nlp, x);
        NLPModels.jac_coord(nlp, x);
        NLPModels.hess_coord(nlp, x, y))
        row(p, cfg, :first_eval, t)
        function solve()
            return OC.solve(
                ocp;
                init=OCP.initial_guess(pb),
                discretizer=OC.Collocation(; grid_size=GRID, scheme=:trapeze),
                modeler=MODELERS[m](),
                solver=solver(s),
                display=false,
            )
        end
        t = @timed solve()
        row(p, cfg, :solve_1, t)
        t = @timed solve()
        row(p, cfg, :solve_2, t)
    end
end
