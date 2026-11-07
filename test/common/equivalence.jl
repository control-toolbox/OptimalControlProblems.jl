# Equivalence of the NLPs of the modelings of a problem (#371, #166): the JuMP NLP and the
# Exa NLP of OptimalControl against CTDirect's (ADNLP). Used by `suite/comparison/`: the
# trapezoidal rule for every problem, every scheme on problems chosen by their features (#401).
module Equivalence

using Test: Test
using JuMP: JuMP
using NLPModels: NLPModels
using NLPModelsJuMP: NLPModelsJuMP
using NLPModelsTest: NLPModelsTest
using OptimalControl: OptimalControl
using OptimalControlProblems: OptimalControlProblems

const OCP = OptimalControlProblems

# The JuMP model of a problem is the NLP of CTDirect (#371, #166), for every scheme and
# ordering. The variables are matched by their names, the constraints by their values at
# random points; the JuMP NLP is then put in CTDirect's order (`PermutedNLP`) and compared
# with `NLPModelsTest.consistent_nlps`: meta, objective, gradient, constraints, Jacobian,
# Hessian of the Lagrangian and their products, as values. The Exa NLP of OptimalControl is
# compared with the same reference: exact for the trapezoidal rule; for the other schemes
# its known gap (#284: one more control, the path constraints at one extreme node) is
# recognised by computation and reported.
const SCHEMES = (:trapeze, :euler, :euler_implicit, :midpoint)
const ORDERINGS = (:interleaved, :blocks)
const GRID_SIZE = 4
const TOL = 1e-10

close_to(a, b) = all(@. isequal(a, b) | (abs(a - b) <= TOL * max(1, abs(a), abs(b))))

# A matrix entry is compared with the size of the matrix: an entry next to large ones carries
# their rounding errors (the Hessian of `rocket` reaches 1e88 at the pseudo-random point).
function close_to(A::AbstractMatrix, B::AbstractMatrix)
    size(A) == size(B) || return false
    scale = max(1, maximum(abs, A; init=0.0), maximum(abs, B; init=0.0))
    return all(@. isequal(A, B) | (abs(A - B) <= TOL * scale))
end

"""
    nonzeros(nlp)

Number of distinct entries of the Jacobian and of the Hessian of the Lagrangian of `nlp`.
Count the nonzeros with this function, never with `nlp.meta.nnzj` or `nlp.meta.nnzh`: these
are the lengths of `jac_coord` and `hess_coord`, which may repeat an entry
(`MathOptNLPModel` returns one per constraint and term, #372).
"""
function nonzeros(nlp)
    distinct(rows, cols) = length(unique(zip(rows, cols)))
    return (
        jacobian=distinct(NLPModels.jac_structure(nlp)...),
        hessian=distinct(NLPModels.hess_structure(nlp)...),
    )
end

# -----------------------------------------------------------------------------
# Variables: each NLP is read through the names of its variables in CTDirect's order.
# -----------------------------------------------------------------------------
nodes(model) = length(model[Symbol(first(model[:state_components]))]) - 1
controls(model) = length(model[Symbol(first(model[:control_components]))])

"""Names of the variables in CTDirect's order: node by node, the state then the control
while there is one (`M` controls), then the variable."""
function ctdirect_names(model, M)
    names = String[]
    for i in 0:nodes(model)
        append!(names, ["$c[$i]" for c in model[:state_components]])
        i < M && append!(names, ["$c[$i]" for c in model[:control_components]])
    end
    return append!(names, model[:variable_components])
end

"""Names of the variables of the Exa model of OptimalControl, with `M` controls: the
variable, the state at every node, the controls. Its control `j` is CTDirect's control
`j - offset`."""
function exa_names(model, M, offset)
    names = copy(model[:variable_components])
    for i in 0:nodes(model)
        append!(names, ["$c[$i]" for c in model[:state_components]])
    end
    for j in 0:(M - 1)
        append!(names, ["$c[$(j - offset)]" for c in model[:control_components]])
    end
    return names
end

"""Column, among `names`, of each variable of `reference`."""
function columns(names, reference)
    position = Dict(n => k for (k, n) in enumerate(names))
    return [position[n] for n in reference]
end

# -----------------------------------------------------------------------------
# Constraints: matched by their margins at the points, which depend neither on the order of
# the rows nor on a constant moved into the bounds (JuMP does that for an affine constraint).
# -----------------------------------------------------------------------------
"""Margins `c - l` and `u - c` of the constraints at `x`, one row per constraint."""
function margins(nlp, x)
    c, m = NLPModels.cons(nlp, x), nlp.meta
    return [c .- m.lcon m.ucon .- c]
end

"""
Key of each constraint of `nlp`: its margins at the points `xs` and its Jacobian row at the
first point, on the columns `cols` (CTDirect's order). Two constraints with the same values
at the points but different derivatives get different keys.
"""
function signatures(nlp, xs, cols)
    M = reduce(hcat, [margins(nlp, x) for x in xs])
    J = Matrix(NLPModels.jac(nlp, xs[1]))[:, cols]
    key(v) = round(v; sigdigits=8) + 0.0     # + 0.0: -0.0 and 0.0 are the same key
    return [Tuple(key.([M[i, :]; J[i, :]])) for i in 1:(nlp.meta.ncon)]
end

"""For each constraint of the reference, the matching constraint of the other NLP (0: none)."""
function match_rows(reference, other)
    pool = Dict{Any,Vector{Int}}()
    for (i, s) in enumerate(other)
        push!(get!(pool, s, Int[]), i)
    end
    return [isempty(get(pool, s, Int[])) ? 0 : pop!(pool[s]) for s in reference]
end

"""
Compare `other` with `reference` (CTDirect's NLP) at the points `xs` and multipliers `y`,
`cols[k]` being the column of the variable `k` of the reference. Returns the checks, the
constraints of each NLP left unmatched, and the columns of `other` that the reference does
not have, with whether they enter the matched part of `other`.
"""
function compare(reference, other, cols, xs, y)
    extra = setdiff(1:(other.meta.nvar), cols)
    lift(x) = (z = fill(0.5, other.meta.nvar); z[cols] = x; z)
    rows = match_rows(
        signatures(reference, xs, 1:(reference.meta.nvar)),
        signatures(other, lift.(xs), cols),
    )
    matched = findall(!iszero, rows)
    r = rows[matched]
    ok = Dict(
        :ncon => reference.meta.ncon == other.meta.ncon,
        :lvar => close_to(reference.meta.lvar, other.meta.lvar[cols]),
        :uvar => close_to(reference.meta.uvar, other.meta.uvar[cols]),
        :x0 => close_to(reference.meta.x0, other.meta.x0[cols]),
    )
    used = false
    yr, yo = zeros(reference.meta.ncon), zeros(other.meta.ncon)
    yr[matched], yo[r] = y[matched], y[matched]
    for (k, x) in enumerate(xs)
        z = lift(x)
        go = NLPModels.grad(other, z)
        Jr, Jo = NLPModels.jac(reference, x), NLPModels.jac(other, z)
        Hr = Matrix(NLPModels.hess(reference, x, yr))
        Ho = Matrix(NLPModels.hess(other, z, yo))
        check(name, value) = (ok[Symbol(name, k)] = value)
        check(:obj, close_to(NLPModels.obj(reference, x), NLPModels.obj(other, z)))
        check(:grad, close_to(NLPModels.grad(reference, x), go[cols]))
        check(:cons, close_to(margins(reference, x)[matched, :], margins(other, z)[r, :]))
        check(:jac, close_to(Matrix(Jr[matched, :]), Matrix(Jo[r, cols])))
        check(:hess, close_to(Hr, Ho[cols, cols]))
        used |=
            any(!iszero, go[extra]) ||
            any(!iszero, Jo[r, extra]) ||
            any(!iszero, Ho[extra, :])
    end
    unmatched = (reference=findall(iszero, rows), other=setdiff(1:(other.meta.ncon), r))
    return (ok=ok, rows=rows, unmatched=unmatched, extra=extra, used=used)
end

"""The checks that fail, and the unmatched constraints and extra columns: empty if exact."""
function differences(c)
    failed = sort!([k for (k, v) in c.ok if !v])
    isempty(c.unmatched.reference) || push!(failed, :unmatched_reference)
    isempty(c.unmatched.other) || push!(failed, :unmatched_other)
    isempty(c.extra) || push!(failed, :extra_variables)
    return failed
end

# -----------------------------------------------------------------------------
# An NLP seen in CTDirect's order, for `consistent_nlps`.
# -----------------------------------------------------------------------------
"""
The NLP `inner` with its variables `cols` and constraints `rows`, in this order, and the
variables recentred: the variable `x` of this NLP is `center + scale * x` in `inner`.
`shift` is added to each constraint and its bounds (JuMP moves the constant of an affine
constraint into its bounds). A column of `inner` not in `cols` keeps its initial value.

`consistent_nlps` evaluates at `x = ±10`, outside the domain of several problems (`NaN`),
and compares with an absolute tolerance; recentred on the initial point with a small scale,
it evaluates where the values have the scale of the problem.
"""
struct PermutedNLP{M<:NLPModels.AbstractNLPModel} <:
       NLPModels.AbstractNLPModel{Float64,Vector{Float64}}
    meta::NLPModels.NLPModelMeta{Float64,Vector{Float64}}
    counters::NLPModels.Counters
    inner::M
    cols::Vector{Int}
    rows::Vector{Int}
    shift::Vector{Float64}
    center::Vector{Float64}
    scale::Float64
    jac::NTuple{3,Vector{Int}}   # rows, columns, position of each entry of `inner`
    hess::NTuple{3,Vector{Int}}
end

# The entries `(rows, cols)` of `inner` mapped by `entry` (`nothing`: dropped), merged.
function coordinates(rows, cols, entry)
    keys, index, position = Tuple{Int,Int}[],
    Dict{Tuple{Int,Int},Int}(),
    zeros(Int, length(rows))
    for k in eachindex(rows)
        e = entry(rows[k], cols[k])
        isnothing(e) && continue
        position[k] = get!(() -> (push!(keys, e); length(keys)), index, e)
    end
    return first.(keys), last.(keys), position
end

function PermutedNLP(inner, cols, rows, shift, center, scale)
    col, row = zeros(Int, inner.meta.nvar), zeros(Int, inner.meta.ncon)
    col[cols], row[rows] = eachindex(cols), eachindex(rows)
    jac = coordinates(
        NLPModels.jac_structure(inner)...,
        (i, j) -> row[i] == 0 || col[j] == 0 ? nothing : (row[i], col[j]),
    )
    hess = coordinates(
        NLPModels.hess_structure(inner)...,
        (i, j) -> col[i] == 0 || col[j] == 0 ? nothing : minmax(col[i], col[j])[[2, 1]],
    )
    m = inner.meta
    meta = NLPModels.NLPModelMeta(
        length(cols);
        ncon=length(rows),
        x0=(m.x0[cols] .- center) ./ scale,
        lvar=(m.lvar[cols] .- center) ./ scale,
        uvar=(m.uvar[cols] .- center) ./ scale,
        lcon=m.lcon[rows] .+ shift,
        ucon=m.ucon[rows] .+ shift,
        nnzj=length(jac[1]),
        nnzh=length(hess[1]),
        name="$(m.name) in CTDirect's order",
    )
    return PermutedNLP(
        meta, NLPModels.Counters(), inner, cols, rows, shift, center, scale, jac, hess
    )
end

function lift(nlp::PermutedNLP, x)
    z = copy(nlp.inner.meta.x0)
    z[nlp.cols] = nlp.center .+ nlp.scale .* x
    return z
end

function merge!(vals, position, inner)
    fill!(vals, 0)
    for k in eachindex(position)
        position[k] > 0 && (vals[position[k]] += inner[k])
    end
    return vals
end

function jac_values(nlp::PermutedNLP, x)
    vals = zeros(nlp.meta.nnzj)
    merge!(vals, nlp.jac[3], NLPModels.jac_coord(nlp.inner, lift(nlp, x)))
    return vals .*= nlp.scale
end

function hess_values(nlp::PermutedNLP, x, y, obj_weight)
    yi = zeros(nlp.inner.meta.ncon)
    yi[nlp.rows] = y
    vals = zeros(nlp.meta.nnzh)
    inner = NLPModels.hess_coord(nlp.inner, lift(nlp, x), yi; obj_weight=obj_weight)
    merge!(vals, nlp.hess[3], inner)
    return vals .*= nlp.scale^2
end

function NLPModels.obj(nlp::PermutedNLP, x::AbstractVector)
    NLPModels.increment!(nlp, :neval_obj)
    return NLPModels.obj(nlp.inner, lift(nlp, x))
end

function NLPModels.grad!(nlp::PermutedNLP, x::AbstractVector, g::AbstractVector)
    NLPModels.increment!(nlp, :neval_grad)
    return g .= nlp.scale .* NLPModels.grad(nlp.inner, lift(nlp, x))[nlp.cols]
end

function NLPModels.cons_nln!(nlp::PermutedNLP, x::AbstractVector, c::AbstractVector)
    NLPModels.increment!(nlp, :neval_cons_nln)
    return c .= NLPModels.cons(nlp.inner, lift(nlp, x))[nlp.rows] .+ nlp.shift
end

function NLPModels.jac_nln_structure!(
    nlp::PermutedNLP, rows::AbstractVector{<:Integer}, cols::AbstractVector{<:Integer}
)
    rows .= nlp.jac[1]
    cols .= nlp.jac[2]
    return rows, cols
end

function NLPModels.jac_nln_coord!(nlp::PermutedNLP, x::AbstractVector, vals::AbstractVector)
    NLPModels.increment!(nlp, :neval_jac_nln)
    return vals .= jac_values(nlp, x)
end

function NLPModels.jprod_nln!(
    nlp::PermutedNLP, x::AbstractVector, v::AbstractVector, Jv::AbstractVector
)
    NLPModels.increment!(nlp, :neval_jprod_nln)
    return NLPModels.coo_prod!(nlp.jac[1], nlp.jac[2], jac_values(nlp, x), v, Jv)
end

function NLPModels.jtprod_nln!(
    nlp::PermutedNLP, x::AbstractVector, v::AbstractVector, Jtv::AbstractVector
)
    NLPModels.increment!(nlp, :neval_jtprod_nln)
    return NLPModels.coo_prod!(nlp.jac[2], nlp.jac[1], jac_values(nlp, x), v, Jtv)
end

function NLPModels.hess_structure!(
    nlp::PermutedNLP, rows::AbstractVector{<:Integer}, cols::AbstractVector{<:Integer}
)
    rows .= nlp.hess[1]
    cols .= nlp.hess[2]
    return rows, cols
end

function NLPModels.hess_coord!(
    nlp::PermutedNLP,
    x::AbstractVector,
    y::AbstractVector,
    vals::AbstractVector;
    obj_weight::Real=1.0,
)
    NLPModels.increment!(nlp, :neval_hess)
    return vals .= hess_values(nlp, x, y, obj_weight)
end

function NLPModels.hprod!(
    nlp::PermutedNLP,
    x::AbstractVector,
    y::AbstractVector,
    v::AbstractVector,
    Hv::AbstractVector;
    obj_weight::Real=1.0,
)
    NLPModels.increment!(nlp, :neval_hprod)
    vals = hess_values(nlp, x, y, obj_weight)
    return NLPModels.coo_sym_prod!(nlp.hess[1], nlp.hess[2], vals, v, Hv)
end

"""Shift of each matched constraint of `other`: the difference of a finite bound."""
function shifts(reference, other, rows)
    l, u = reference.meta.lcon, reference.meta.ucon
    lo, uo = other.meta.lcon[rows], other.meta.ucon[rows]
    return [
        if isfinite(l[k])
            l[k] - lo[k]
        elseif isfinite(u[k])
            u[k] - uo[k]
        else
            0.0
        end for k in eachindex(rows)
    ]
end

# The ADNLP model of CTDirect builds no AD backend for the products (`jprod`, `jtprod`,
# `hprod`): they are not compared. The Jacobian and the Hessian they come from are.
const NOT_BUILT = [
    NLPModels.jprod,
    NLPModels.jtprod,
    NLPModels.hprod,
    NLPModels.jth_hess,
    NLPModels.jth_hess_coord,
    NLPModels.jth_hprod,
    NLPModels.ghjvprod,
]

"""
Exact comparison: the checks above, then `consistent_nlps` in CTDirect's order: meta and
functions. Not run: the counters, the finite-difference checks of the derivatives, the
quasi-Newton and slack models; they test the implementations (ADNLPModels, NLPModelsJuMP,
NLPModelsModifiers), not the equivalence of the formulations.

`allowed` lists checks whose failure is returned instead of failing the test (the initial
point of Exa, see `test_exa`); the meta is then checked by `compare` only.
"""
function test_exact(reference, other, cols, ref; allowed=Symbol[])
    c = compare(reference, other, cols, ref.xs, ref.y)
    tolerated = intersect(differences(c), allowed)
    Test.@test setdiff(differences(c), allowed) == Symbol[]
    setdiff(differences(c), allowed) == Symbol[] || return tolerated
    n, m = reference.meta.nvar, reference.meta.ncon
    # the initial point becomes 1, not 0: `consistent_meta` compares it with a relative
    # tolerance only (JuliaSmoothOptimizers/NLPModelsTest.jl#150)
    scale = 0.001
    center = reference.meta.x0 .- scale
    recentred = PermutedNLP(reference, collect(1:n), collect(1:m), zeros(m), center, scale)
    permuted = PermutedNLP(
        other, cols, c.rows, shifts(reference, other, c.rows), center, scale
    )
    NLPModelsTest.consistent_nlps(
        [recentred, permuted];
        exclude=NOT_BUILT,
        test_meta=isempty(tolerated),
        test_counters=false,
        test_derivative=false,
        test_qn=false,
        test_slack=false,
    )
    return tolerated
end

"""
CTDirect's NLP (ADNLP) of `problem` for `scheme`, the Exa NLP of OptimalControl, the OCP,
the points (initial point and a pseudo-random one, in CTDirect's order) and multipliers.
"""
function reference(problem, scheme)
    pb = getfield(OCP, problem)(OCP.OptimalControlBackend())
    ocp = OCP.model(pb)
    docp = OptimalControl.discretize(
        ocp, OptimalControl.Collocation(; grid_size=GRID_SIZE, scheme=scheme)
    )
    init = OptimalControl.build_initial_guess(ocp, OCP.initial_guess(pb))
    adnlp = OptimalControl.nlp_model(docp, init, OptimalControl.ADNLP())
    exa = OptimalControl.nlp_model(docp, init, OptimalControl.Exa())
    n, m = adnlp.meta.nvar, adnlp.meta.ncon
    xs = [adnlp.meta.x0, [0.6 + 0.8 * ((7919 * i) % 1009) / 1009 for i in 1:n]]
    y = [-1 + 2 * ((104729 * i) % 997) / 997 for i in 1:m]
    return (; adnlp, exa, ocp, xs, y)
end

"""
The constraints of CTDirect's NLP that are the path constraints at the first or the last
node: in CTDirect's order, the dynamics of step 1 then the path constraints at node 0, ...,
the path constraints at node N, then the boundary conditions.
"""
function extreme_path_rows(ocp)
    n = OptimalControl.state_dimension(ocp)
    np = OptimalControl.dim_path_constraints_nl(ocp)
    N = GRID_SIZE
    return ((n + 1):(n + np), (N * (n + np) + 1):(N * (n + np) + np))
end

function test_jump(problem, scheme, ref)
    for ordering in ORDERINGS
        Test.@testset "JuMP, $scheme, $ordering" begin
            model = getfield(OCP, problem)(
                OCP.JuMPBackend(); grid_size=GRID_SIZE, scheme=scheme, ordering=ordering
            )
            jump = NLPModelsJuMP.MathOptNLPModel(model)
            Test.@test jump.meta.nvar == ref.adnlp.meta.nvar
            jump.meta.nvar == ref.adnlp.meta.nvar || continue
            names = JuMP.name.(JuMP.all_variables(model))
            cols = columns(names, ctdirect_names(model, controls(model)))
            test_exact(ref.adnlp, jump, cols, ref)
        end
    end
end

# The initial point of the Exa model of OptimalControl may differ from ADNLP's for a
# time-dependent initial guess (`chain`: the control at the last node): a difference
# between the two modelers of OptimalControl, not of the equations. It is reported.
const EXA_ALLOWED = [:x0]

function report_exa(problem, scheme, text)
    return Main.TestStatus.report!(
        "NLP equivalence, $scheme", problem, "OptimalControl ADNLP vs Exa", :valid, text
    )
end

function test_exa(problem, scheme, ref)
    Test.@testset "Exa, $scheme" begin
        model = getfield(OCP, problem)(
            OCP.JuMPBackend(); grid_size=GRID_SIZE, scheme=scheme
        )
        exa, M = ref.exa, controls(model)
        m = length(model[:control_components])
        extra_controls = (exa.meta.nvar - ref.adnlp.meta.nvar) ÷ m
        notes = String[]
        if extra_controls == 0
            cols = columns(exa_names(model, M, 0), ctdirect_names(model, M))
            tolerated = test_exact(ref.adnlp, exa, cols, ref; allowed=EXA_ALLOWED)
            :x0 in tolerated && push!(notes, "initial point differs from ADNLP's")
        else
            # the known gap: one more control, at either end, that enters neither the
            # dynamics nor the cost; only the path constraints at one extreme node differ
            gap = nothing
            for offset in (0, 1)
                names = exa_names(model, M + 1, offset)
                cols = columns(names, ctdirect_names(model, M))
                c = compare(ref.adnlp, exa, cols, ref.xs, ref.y)
                rows = sort(c.unmatched.reference)
                # compared by their keys: a constraint of the control only is the same at
                # nodes 0 and 1 for implicit Euler, so which copy is left unmatched is free
                keys = signatures(ref.adnlp, ref.xs, 1:(ref.adnlp.meta.nvar))
                same(r) = sort(keys[rows]) == sort(keys[collect(r)])
                known =
                    extra_controls == 1 &&
                    all(v for (k, v) in c.ok if k ∉ EXA_ALLOWED) &&
                    !c.used &&
                    length(c.unmatched.other) == length(rows) &&
                    (isempty(rows) || any(same, extreme_path_rows(ref.ocp)))
                known && (gap = (offset=offset, rows=rows, x0=c.ok[:x0]))
            end
            Test.@test !isnothing(gap)
            if !isnothing(gap)
                node = gap.offset == 0 ? "N" : "0"
                push!(
                    notes,
                    "known gap: N+1 controls, " * (
                        if isempty(gap.rows)
                            "no constraint differs"
                        else
                            "path constraints at node $node differ"
                        end
                    ),
                )
                gap.x0 || push!(notes, "initial point differs from ADNLP's")
            end
        end
        isempty(notes) || report_exa(problem, scheme, "Exa: " * join(notes, "; "))
    end
end

"""Every comparison of `problem` for the schemes `schemes`."""
function test_problem(problem, schemes)
    for scheme in schemes
        ref = reference(problem, scheme)
        test_jump(problem, scheme, ref)
        test_exa(problem, scheme, ref)
    end
end

"""
`MathOptNLPModel` repeats Hessian entries: `nnzh` is not a count of nonzeros. If this fails,
it no longer does, and `nonzeros` may be simplified.
"""
function test_nonzeros()
    Test.@testset "nnzh counts repeated entries" begin
        repeated = map(OCP.problems(OCP.JuMPBackend())) do problem
            jump = NLPModelsJuMP.MathOptNLPModel(
                getfield(OCP, problem)(OCP.JuMPBackend(); grid_size=GRID_SIZE)
            )
            jump.meta.nnzh > nonzeros(jump).hessian
        end
        Test.@test any(repeated)
    end
end

end # module Equivalence
