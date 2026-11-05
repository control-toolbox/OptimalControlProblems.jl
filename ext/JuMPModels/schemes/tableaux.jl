# The schemes as Butcher tableaux (#369). Their properties are traits computed from the
# tableau, never declared; the names follow the CTDirect roadmap (CTDirect.jl#610, discussion
# #609, §2.2 and §4), so that this private mirror can be replaced by CTDirect's `Schemes`.
#
# The JuMP models have no stage unknowns: every stage must be a node of the step (`X_i` or
# `X_{i+1}`) or, for a one-stage tableau, an affine combination of both ("node form").

"""
$(TYPEDEF)

A Runge–Kutta scheme given by its Butcher tableau `(A, b, c)`, with `s` stages.
"""
struct ButcherTableau
    name::Symbol
    A::Matrix{Float64}
    b::Vector{Float64}
    c::Vector{Float64}
end

# The schemes of `OptimalControl.Collocation` that the JuMP models reproduce.
const TABLEAUX = (
    trapeze=ButcherTableau(:trapeze, [0 0; 0.5 0.5], [0.5, 0.5], [0, 1]),
    euler=ButcherTableau(:euler, fill(0.0, 1, 1), [1], [0]),
    euler_implicit=ButcherTableau(:euler_implicit, fill(1.0, 1, 1), [1], [1]),
    midpoint=ButcherTableau(:midpoint, fill(0.5, 1, 1), [1], [0.5]),
)

"""
$(TYPEDSIGNATURES)

The tableau of `scheme`, one of the keys of `TABLEAUX`.

# Throws

- `CTBase.Exceptions.IncorrectArgument`: If `scheme` is not one of them.
"""
function tableau(scheme::Symbol)
    haskey(TABLEAUX, scheme) || throw(
        CTBase.Exceptions.IncorrectArgument(
            "Unknown scheme for the JuMP models";
            got=":$scheme",
            expected=join((":$k" for k in keys(TABLEAUX)), ", "),
            suggestion="use one of the schemes of OptimalControl.Collocation listed above",
            context="JuMP model of OptimalControlProblems",
        ),
    )
    return TABLEAUX[scheme]
end

stage_count(tab::ButcherTableau) = length(tab.b)
is_explicit(tab::ButcherTableau) = all(iszero(tab.A[j, k]) for j in 1:stage_count(tab), k in j:stage_count(tab))

"""Last row of `A` equal to `b` and `c_s = 1`: the last stage is the next node `X_{i+1}`."""
is_stiffly_accurate(tab::ButcherTableau) = tab.A[end, :] == tab.b && tab.c[end] == 1

"""First row of `A` zero and `c_1 = 0`: the first stage is the node `X_i`."""
has_explicit_first_stage(tab::ButcherTableau) = iszero(tab.A[1, :]) && tab.c[1] == 0

"""First Same As Last: the last stage of a step is the first stage of the next one."""
is_fsal(tab::ButcherTableau) = is_stiffly_accurate(tab) && has_explicit_first_stage(tab)

# Reuse of the stages across steps: with FSAL, the dynamics are evaluated once per node.
abstract type StageReuse <: CTBase.Traits.AbstractTrait end
struct FirstSameAsLast <: StageReuse end
struct NoStageReuse <: StageReuse end
stage_reuse(tab::ButcherTableau) = is_fsal(tab) ? FirstSameAsLast() : NoStageReuse()

# Control parametrization (CTDirect roadmap §2.2): one control per step; the stagewise
# controls of the implicit Runge–Kutta schemes need stage unknowns.
abstract type ControlParametrization <: CTBase.Traits.AbstractTrait end
struct StepwiseControl <: ControlParametrization end
control_parametrization(::ButcherTableau) = StepwiseControl()

"""
The controls of a FSAL tableau live at the nodes, shared by neighbouring steps: one more
control than steps (CTDirect's `_final_control`).
"""
has_final_control(tab::ButcherTableau) = is_fsal(tab)

"""Number of controls on a grid of `N` steps."""
control_count(tab::ButcherTableau, N::Int) = has_final_control(tab) ? N + 1 : N

"""
$(TYPEDSIGNATURES)

The position `θ_j` of each stage in the step, `Z_j = X_i + θ_j (X_{i+1} - X_i)`.

# Throws

- `CTBase.Exceptions.NotImplemented`: If a stage is not of this form (an interior stage of a
  tableau of several stages): it needs stage unknowns, which the JuMP models do not have.
"""
function stage_positions(tab::ButcherTableau)
    s = stage_count(tab)
    s == 1 && return [tab.A[1, 1] / tab.b[1]]
    θ = map(1:s) do j
        iszero(tab.A[j, :]) && return 0.0
        tab.A[j, :] == tab.b && return 1.0
        throw(
            CTBase.Exceptions.NotImplemented(
                "Stage $j of the tableau $(tab.name) is not a node of the step";
                required_method="stage unknowns in the JuMP models",
                suggestion="use a tableau whose stages are nodes, or one with a single stage",
                context="JuMP model of OptimalControlProblems",
            ),
        )
    end
    return θ
end

"""
$(TYPEDSIGNATURES)

The control (0-based) used at the node `k` of a grid of `N` steps, by the path constraints.
At the nodes for a tableau with final control; otherwise the control of the step that
starts at the node, the last node taking the last one, except when the only stage is at the
end of the step (implicit Euler): the control of the step that ends at the node, the first
node taking the first one (CTDirect's conventions).
"""
function control_at_node(tab::ButcherTableau, k::Int, N::Int)
    has_final_control(tab) && return k
    return all(==(1), tab.c) ? max(k - 1, 0) : min(k, N - 1)
end

"""The control (0-based) of stage `j` of step `i` (0-based)."""
function control_at_stage(tab::ButcherTableau, θ, i::Int, j::Int)
    return has_final_control(tab) ? i + Int(θ[j]) : i
end
