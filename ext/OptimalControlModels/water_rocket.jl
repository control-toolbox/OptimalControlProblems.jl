"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** for the Water Rocket (Propelled Phase).  
The goal is to drive the rocket during the water ejection phase.  
The objective is to maximise the altitude at the end of the water ejection.

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `grid_size::Int=100`: (Keyword) Number of discretisation points for the direct transcription grid.

# Returns

- `docp`: The direct optimal control problem object representing the Water Rocket problem.

# Example

```julia-repl
julia> using OptimalControlProblems

julia> docp = OptimalControlProblems.water_rocket(OptimalControlBackend(); N=100);
```
"""
function OptimalControlProblems.water_rocket(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=grid_size_data(:water_rocket),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:water_rocket, parameters)
    g = params[:g]
    rho_w = params[:rho_w]
    p_a = params[:p_a]
    k = params[:k]
    V_b = params[:V_b]
    A_out = params[:A_out]
    S = params[:S]
    C_d = params[:C_d]
    rho_a = params[:rho_a]
    m_empty = params[:m_empty]
    t0 = params[:t0]
    r_t0 = params[:r_t0]
    h_t0 = params[:h_t0]
    v_t0 = params[:v_t0]
    p_t0 = params[:p_t0]
    tf_start = params[:tf_start]
    Vw0_start = params[:Vw0_start]
    gamma0_start = params[:gamma0_start]

    # model
    ocp = @def begin
        tf ∈ R, variable
        Vw0 ∈ R, variable
        γ0 ∈ R, variable
        t ∈ [t0, tf], time
        x = (r, h, v, γ, p, Vw) ∈ R⁶, state
        u ∈ R, control # dummy

        # constraints
        0.001 ≤ tf ≤ 1.0
        0.1e-3 ≤ Vw0 ≤ 1.9e-3
        0.1 ≤ γ0 ≤ 1.5
        Vw(tf) == 0.0
        h(t) ≥ 0.0

        # initial conditions
        r(t0) == r_t0
        h(t0) == h_t0
        v(t0) == v_t0
        γ(t0) == γ0
        p(t0) == p_t0
        Vw(t0) == Vw0

        # dynamics
        ṙ(t) == v(t) * cos(γ(t))
        ḣ(t) == v(t) * sin(γ(t))
        
        # Intermediate calculations (implicitly in dynamics)
        # v_out = sqrt(2*(p - p_a)/rho_w)
        # Vẇ = -v_out * A_out
        # ṗ = k * p * Vẇ / (V_b - Vw)
        # T = 2 * A_out * (p - p_a)
        # m = m_empty + rho_w * Vw
        # D = 0.5 * rho_a * v^2 * S * C_d
        # v̇ = (T - D - m * g * sin(γ)) / m
        # γ̇ = - (g * cos(γ)) / v

        Vẇ(t) == -sqrt(2 * (p(t) - p_a) / rho_w) * A_out
        ṗ(t) == k * p(t) * (-sqrt(2 * (p(t) - p_a) / rho_w) * A_out) / (V_b - Vw(t))
        v̇(t) == (2 * A_out * (p(t) - p_a) - 0.5 * rho_a * v(t)^2 * S * C_d - (m_empty + rho_w * Vw(t)) * g * sin(γ(t))) / (m_empty + rho_w * Vw(t))
        γ̇(t) == - (g * cos(γ(t))) / v(t)

        # objective
        h(tf) → max
    end

    # initial guess
    init = (state=[0.0, 0.0, 1.0, 0.785, 7.0e5, 1e-3], control=0.0, variable=[tf_start, Vw0_start, gamma0_start])

    # discretise the optimal control problem
    docp = direct_transcription(
        ocp,
        description...;
        lagrange_to_mayer=false,
        init=init,
        grid_size=grid_size,
        disc_method=:trapeze,
        kwargs...,
    )

    return docp
end
