"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** for the Aircraft Balanced Field Length Calculation (Takeoff phase).  
The objective is to minimise the final range to reach a specified altitude (35 ft) with one engine out.  
The problem formulation is based on the Dymos balanced field example.

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `grid_size::Int=200`: (Keyword) Number of discretisation points for the direct transcription grid.

# Returns

- `docp`: The direct optimal control problem object representing the problem.
- `nlp`: The corresponding nonlinear programming model.

# Example

```julia-repl
julia> using OptimalControlProblems
julia> docp = OptimalControlProblems.balanced_field(OptimalControlBackend());
```
"""
function OptimalControlProblems.balanced_field(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=grid_size_data(:balanced_field),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:balanced_field, parameters)
    t0 = params[:t0]
    m = params[:m]
    g = params[:g]
    ρ = params[:ρ]
    S = params[:S]
    CD0 = params[:CD0]
    AR = params[:AR]
    e = params[:e]
    CL0 = params[:CL0]
    CL_max = params[:CL_max]
    h_w = params[:h_w]
    span = params[:span]
    α_max = params[:α_max]
    T = params[:T]
    
    r_t0 = params[:r_t0]
    v_t0 = params[:v_t0]
    h_t0 = params[:h_t0]
    γ_t0 = params[:γ_t0]
    h_tf = params[:h_tf]
    γ_tf = params[:γ_tf]

    α_min = params[:α_min]
    α_max_ctrl = params[:α_max_ctrl]

    # Constants
    b = span / 2.0
    K_nom = 1.0 / (pi * AR * e)

    # model
    ocp = @def begin
        tf ∈ R, variable
        t ∈ [t0, tf], time
        x = (r, v, h, γ) ∈ R⁴, state
        α ∈ R, control

        r(t0) == r_t0
        v(t0) == v_t0
        h(t0) == h_t0
        γ(t0) == γ_t0

        h(tf) == h_tf
        γ(tf) == γ_tf
        
        0 ≤ α(t) ≤ α_max_ctrl # alpha
        tf ≥ 0.1
        h(t) ≥ 0
        v(t) ≥ 1.0

        ẋ(t) == dynamics(x(t), α(t), m, g, ρ, S, CD0, CL0, CL_max, α_max, T, b, K_nom, h_w)

        r(tf) → min
    end

    function dynamics(x, α, m, g, ρ, S, CD0, CL0, CL_max, α_max, T, b, K_nom, h_w)
        r, v, h, γ = x
        
        q = 0.5 * ρ * v^2
        CL = CL0 + (α / α_max) * (CL_max - CL0)
        L = q * S * CL
        
        h_eff = h + h_w
        term_h = 33.0 * abs(h_eff / b)^1.5
        K = K_nom * term_h / (1.0 + term_h)
        D = q * S * (CD0 + K * CL^2)
        
        rdot = v * cos(γ)
        vdot = (T * cos(α) - D) / m - g * sin(γ)
        hdot = v * sin(γ)
        γdot = (T * sin(α) + L) / (m * v) - (g * cos(γ)) / v
        
        return [rdot, vdot, hdot, γdot]
    end

    # initial guess
    tf_guess = 10.0
    xinit = [r_t0, v_t0 + 10.0, h_tf / 2.0, γ_tf]
    uinit = [0.1]
    vinit = [tf_guess]
    init = (state=xinit, control=uinit, variable=vinit)

    # discretise
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
