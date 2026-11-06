"""The **Space Shuttle reentry problem** is a classical benchmark in aerospace optimal control.

Builds the `space_shuttle` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> space_shuttle(JuMPBackend(); grid_size=100)
```

The keyword `scheme` chooses the discretization: `:trapeze` (default), `:euler`,
`:euler_implicit` or `:midpoint`, as in `OptimalControl.Collocation`.
The keyword `ordering` chooses the order of the variables and constraints:
`:interleaved` (default, as in CTDirect) or `:blocks` (grouped by kind).

Default parameters and the modeling status are available through
`problem_metadata(:space_shuttle)` and `OptimalControlProblems.status(:space_shuttle)`.
"""
function OptimalControlProblems.space_shuttle(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:space_shuttle),
    scheme::Symbol=:trapeze,
    ordering::Symbol=:interleaved,
    parameters::Union{Nothing,NamedTuple}=nothing,
    init=nothing,
    kwargs...,
)
    OptimalControlProblems._warn_if_draft(:space_shuttle, :JuMP)

    # Parameters
    params = parameters_data(:space_shuttle, parameters)
    t0 = params[:t0]

    ##
    w = params[:w]
    g₀ = params[:g₀]
    m = w / g₀      # mass (slug)

    ## Aerodynamic and atmospheric forces on the vehicle
    ρ₀ = params[:ρ₀]
    hᵣ = params[:hᵣ]
    Rₑ = params[:Rₑ]
    μ = params[:μ]
    S = params[:S]
    a₀ = params[:a₀]
    a₁ = params[:a₁]
    b₀ = params[:b₀]
    b₁ = params[:b₁]
    b₂ = params[:b₂]

    # 
    tf_l = params[:tf_l]
    tf_u = params[:tf_u]

    ## Initial conditions
    h_t0 = params[:h_t0]
    ϕ_t0 = params[:ϕ_t0]
    θ_t0 = params[:θ_t0]
    v_t0 = params[:v_t0]
    γ_t0 = params[:γ_t0]
    ψ_t0 = params[:ψ_t0]

    # for initial guess
    α_s = params[:α_s]
    β_s = params[:β_s]

    ## Final conditions, the so-called Terminal Area Energy Management (TAEM)
    h_tf = params[:h_tf]
    v_tf = params[:v_tf]
    γ_tf = params[:γ_tf]

    ##
    h_l = params[:h_l]
    ϕ_l = params[:ϕ_l]
    ϕ_u = params[:ϕ_u]
    θ_l = params[:θ_l]
    θ_u = params[:θ_u]
    v_l = params[:v_l]
    γ_l = params[:γ_l]
    γ_u = params[:γ_u]
    ψ_l = params[:ψ_l]
    ψ_u = params[:ψ_u]
    α_l = params[:α_l]
    α_u = params[:α_u]
    β_l = params[:β_l]
    β_u = params[:β_u]

    ## Scalings
    scaling_h = 1e5
    scaling_v = 1e4


    # aerodynamics and motion of the vehicle, at one node, on the scaled state as in the
    # OptimalControl model: `h` and `v` at their true scale inside, their derivatives scaled
    function motion(x, u)
        h, ϕ, θ, v, γ, ψ = x[1] * scaling_h, x[2], x[3], x[4] * scaling_v, x[5], x[6]
        α, β = u
        c_L = a₀ + a₁ * rad2deg(α)
        c_D = b₀ + b₁ * rad2deg(α) + b₂ * rad2deg(α)^2
        ρ = ρ₀ * exp(-h / hᵣ)
        D = 0.5 * c_D * S * ρ * v^2
        L = 0.5 * c_L * S * ρ * v^2
        r = Rₑ + h
        g = μ / r^2
        return [
            v * sin(γ) / scaling_h,
            (v / r) * cos(γ) * sin(ψ) / cos(θ),
            (v / r) * cos(γ) * cos(ψ),
            (-(D / m) - g * sin(γ)) / scaling_v,
            (L / (m * v)) * cos(β) + cos(γ) * ((v / r) - (g / v)),
            (1 / (m * v * cos(γ))) * L * sin(β) +
            (v / (r * cos(θ))) * cos(γ) * sin(ψ) * sin(θ),
        ]
    end

    data = ProblemData(;
        state=[
            Component("scaled_h"; lower=h_l),  # altitude (ft) / scaling_h
            Component("ϕ"; lower=ϕ_l, upper=ϕ_u),  # longitude (rad)
            Component("θ"; lower=θ_l, upper=θ_u),  # latitude (rad)
            Component("scaled_v"; lower=v_l),  # velocity (ft/sec) / scaling_v
            Component("γ"; lower=γ_l, upper=γ_u),  # flight path angle (rad)
            Component("ψ"; lower=ψ_l, upper=ψ_u),  # azimuth (rad)
        ],
        control=[
            Component("α"; lower=α_l, upper=α_u),  # angle of attack (rad)
            Component("β"; lower=β_l, upper=β_u),  # bank angle (rad)
        ],
        variable=[Component("tf"; lower=tf_l, upper=tf_u)],  # final time (sec)
        costate=["∂h", "∂ϕ", "∂θ", "∂v", "∂γ", "∂ψ"],
        times=v -> (t0, v[1]),
        dynamics=(x, u, v, t) -> motion(x, u),
        cost=(I, x0, xf, v) -> -xf[3],
        boundary=[
            (h_t0, (x0, xf, v) -> x0[1], h_t0),
            (ϕ_t0, (x0, xf, v) -> x0[2], ϕ_t0),
            (θ_t0, (x0, xf, v) -> x0[3], θ_t0),
            (v_t0, (x0, xf, v) -> x0[4], v_t0),
            (γ_t0, (x0, xf, v) -> x0[5], γ_t0),
            (ψ_t0, (x0, xf, v) -> x0[6], ψ_t0),
            (h_tf, (x0, xf, v) -> xf[1], h_tf),
            (v_tf, (x0, xf, v) -> xf[4], v_tf),
            (γ_tf, (x0, xf, v) -> xf[5], γ_tf),
        ],
    )
    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems._initial_guess(:space_shuttle, params, init)
    return build(data, grid_size, scheme, ordering, args...; init=init, kwargs...)
end
