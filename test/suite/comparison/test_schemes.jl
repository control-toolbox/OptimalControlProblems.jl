module TestSchemesComparison

using Test: Test
using JuMP: JuMP
using NLPModels: NLPModels
using NLPModelsJuMP: NLPModelsJuMP
using OptimalControl: OptimalControl
using OptimalControlProblems: OptimalControlProblems

const OCP = OptimalControlProblems
const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# The JuMP model of each scheme is the NLP of CTDirect (#369), on two problems chosen by
# their properties: the first one with nonlinear path constraints, the first one with a
# variable (e.g. a free final time). The check of every problem, scheme and ordering is #371.
function problems()
    both = OCP.problems(OCP.JuMPBackend(), OCP.OptimalControlBackend())
    ocp(p) = OCP.model(getfield(OCP, p)(OCP.OptimalControlBackend()))
    with_path = findfirst(p -> OptimalControl.dim_path_constraints_nl(ocp(p)) > 0, both)
    with_variable = findfirst(p -> OptimalControl.variable_dimension(ocp(p)) > 0, both)
    return unique(both[[with_path, with_variable]])
end

const SCHEMES = (:trapeze, :euler, :euler_implicit, :midpoint)
const ORDERINGS = (:interleaved, :blocks)
const GRID_SIZE = 4
const TOL = 1e-10

close_to(a, b) = all(@. isequal(a, b) | (abs(a - b) <= TOL * max(1, abs(a), abs(b))))

"""
The CTDirect (ADNLP) and JuMP NLPs of `problem` for `scheme`, a point `x` in CTDirect's
order and `p`: the variable `k` of the JuMP model is the variable `p[k]` of CTDirect. The
permutation is read from the names of the variables, those of the interleaved JuMP model
being in CTDirect's order (`p` is the identity for `:interleaved`).
"""
function nlps(problem::Symbol, scheme::Symbol, ordering::Symbol)
    build(o) = getfield(OCP, problem)(
        OCP.JuMPBackend(); grid_size=GRID_SIZE, scheme=scheme, ordering=o
    )
    model, reference = build(ordering), build(:interleaved)
    names(m) = JuMP.name.(JuMP.all_variables(m))
    position = Dict(n => k for (k, n) in enumerate(names(reference)))
    p = [position[n] for n in names(model)]
    jump = NLPModelsJuMP.MathOptNLPModel(model)
    pb = getfield(OCP, problem)(OCP.OptimalControlBackend())
    ocp = OCP.model(pb)
    discretizer = OptimalControl.Collocation(; grid_size=GRID_SIZE, scheme=scheme)
    docp = OptimalControl.discretize(ocp, discretizer)
    init = OptimalControl.build_initial_guess(ocp, OCP.initial_guess(pb))
    adnlp = OptimalControl.nlp_model(docp, init, OptimalControl.ADNLP())
    x = [0.6 + 0.8 * ((7919 * i) % 1009) / 1009 for i in 1:(adnlp.meta.nvar)]
    return adnlp, jump, x, p
end

"""
Margins `(c - l, u - c)` of the constraints at `x`, sorted: `MathOptNLPModel` puts the
linear constraints first and JuMP moves the constant of an affine constraint into its
bounds; the margins do not depend on either.
"""
function margins(nlp, x)
    c, m = NLPModels.cons(nlp, x), nlp.meta
    return sort([(c[i] - m.lcon[i], m.ucon[i] - c[i]) for i in 1:(m.ncon)])
end

function test_schemes()
    Test.@testset "JuMP schemes: the NLP of CTDirect" verbose = VERBOSE showtiming =
        SHOWTIMING begin
        selected = problems()
        Test.@test length(selected) == 2
        for problem in selected, scheme in SCHEMES, ordering in ORDERINGS
            Test.@testset "$problem, $scheme, $ordering" begin
                a, b, x, p = nlps(problem, scheme, ordering)
                Test.@test a.meta.nvar == b.meta.nvar
                Test.@test a.meta.ncon == b.meta.ncon
                Test.@test ordering == :blocks || p == 1:(a.meta.nvar)
                Test.@test close_to(a.meta.lvar[p], b.meta.lvar)
                Test.@test close_to(a.meta.uvar[p], b.meta.uvar)
                Test.@test close_to(NLPModels.obj(a, x), NLPModels.obj(b, x[p]))
                Test.@test close_to(NLPModels.grad(a, x)[p], NLPModels.grad(b, x[p]))
                ma, mb = margins(a, x), margins(b, x[p])
                Test.@test close_to(first.(ma), first.(mb))
                Test.@test close_to(last.(ma), last.(mb))
            end
        end
    end
end

end # module TestSchemesComparison

test_schemes() = TestSchemesComparison.test_schemes()
