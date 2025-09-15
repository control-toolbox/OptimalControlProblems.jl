"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** representing a double oscillator system.  
The function defines state and control variables, system dynamics, boundary conditions, and an objective functional to be minimised.  
It uses direct transcription to produce a discretised optimal control problem (DOCP) and the corresponding nonlinear programming (NLP) model.

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `N::Int=500`: (Keyword) Number of discretisation points for the direct transcription grid.

# Returns

- `docp`: The direct optimal control problem object representing the double oscillator system.
- `nlp`: The corresponding nonlinear programming model obtained from the DOCP, suitable for numerical optimisation.

# Example

```julia-repl
julia> using OptimalControlProblems

julia> docp = OptimalControlProblems.double_oscillator(OptimalControlBackend(); N=100);
```

# References

- Coudurier, C., Lepreux, O., & Petit, N. (2018). Optimal bang-bang control of a mechanical double oscillator using averaging methods. *IFAC-PapersOnLine*, 51(2), 49-54. [CLP2018]
- Formulation follows OptimalControl approach to mechanical oscillator trajectory optimisation.
"""
function OptimalControlProblems.double_oscillator(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=grid_size_data(:double_oscillator),
    parameters::Union{Nothing, NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:double_oscillator, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    m1 = params[:m1]
    m2 = params[:m2]
    c = params[:c]
    k1 = params[:k1]
    k2 = params[:k2]
    u_l = params[:u_l]
    u_u = params[:u_u]
    x₁_t0 = params[:x₁_t0]
    x₂_t0 = params[:x₂_t0]

    # model
    ocp = @def begin
        t ∈ [t0, tf], time
        x ∈ R⁴, state
        u ∈ R, control
        
        u_l ≤ u(t) ≤ u_u, (u_c)
        x₁(t0) == x₁_t0, (x₁_t0)
        x₂(t0) == x₂_t0, (x₂_t0)

        ẋ(t) == dynamics(x(t), u(t), F(t))

        0.5 * ∫(x₁(t)^2 + x₂(t)^2 + u(t)^2) → min
    end

    function F(t)
        return sin((t - t0) * 2π / (tf - t0))
    end

    function dynamics(x, u, F)
        x₁, x₂, x₃, x₄ = x
        dx₁ = x₃
        dx₂ = x₄
        dx₃ = -(k1 + k2) / m1 * x₁ + k2 / m1 * x₂ + 1 / m1 * F
        dx₄ = k2 / m2 * x₁ - k2 / m2 * x₂ - c * (1 - u) / m2 * x₄
        return [dx₁, dx₂, dx₃, dx₄]
    end

    # initial guess
    xinit = [0.1, 0.1, 0.1, 0.1]  # [x₁, x₂, x₃, x₄]
    uinit = [0.1]  # [u]
    init = (state=xinit, control=uinit)

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
