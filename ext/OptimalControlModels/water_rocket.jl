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
    Cd = params[:Cd]
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
        w = (tf, Vw0, gamma0) ∈ R³, variable
        t ∈ [t0, tf], time
        x ∈ R⁶, state
        u ∈ R, control # dummy

        # constraints
        0.001 ≤ tf ≤ 1.0
        0.1e-3 ≤ Vw0 ≤ 1.9e-3
        0.1 ≤ gamma0 ≤ 1.5
        x[6](tf) == 0.0
        x[2](t) ≥ 0.0

        # initial conditions
        x[1](t0) == r_t0
        x[2](t0) == h_t0
        x[3](t0) == v_t0
        x[4](t0) == gamma0
        x[5](t0) == p_t0
        x[6](t0) == Vw0

        # dynamics
        # x[1]=r, x[2]=h, x[3]=v_mag, x[4]=gamma, x[5]=p, x[6]=Vw
        ẋ(t) == [
            x[3](t) * cos(x[4](t)),
            x[3](t) * sin(x[4](t)),
            (2 * A_out * (x[5](t) - p_a) - 0.5 * rho_a * (x[3](t)^2) * Cd * S - (m_empty + rho_w * x[6](t)) * g * sin(x[4](t))) / (m_empty + rho_w * x[6](t)),
            - (g * cos(x[4](t))) / x[3](t),
            k * x[5](t) * (-sqrt(2 * (x[5](t) - p_a) / rho_w) * A_out) / (V_b - x[6](t)),
            -sqrt(2 * (x[5](t) - p_a) / rho_w) * A_out
        ]

        # objective
        x[2](tf) → max
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
