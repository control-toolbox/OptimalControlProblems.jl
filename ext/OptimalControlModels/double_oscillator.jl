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

julia> docp, nlp = OptimalControlProblems.double_oscillator(OptimalControlBackend(); N=100);

julia> docp
DOCP object with 100 discretisation points

julia> nlp
NLP model corresponding to the double oscillator problem
```

# References

- Coudurier, C., Lepreux, O., & Petit, N. (2018). Optimal bang-bang control of a mechanical double oscillator using averaging methods. *IFAC-PapersOnLine*, 51(2), 49-54. [CLP2018]
- Formulation follows OptimalControl approach to mechanical oscillator trajectory optimisation.
"""
function OptimalControlProblems.double_oscillator(::OptimalControlBackend; N::Int=500)

    # parameters
    m1 = 100    # [kg]
    m2 = 2      # [kg]
    c = 0.5     # [Ns/m]
    k1 = 100    # [N/m]
    k2 = 3      # [N/m]
    tf = 2π

    # model
    ocp = @def begin
        t ∈ [0, tf], time
        x ∈ R⁴, state
        u ∈ R, control

        -1 ≤ u(t) ≤ 1, (u_con)

        x₁(0) == 0, (x1_con)
        x₂(0) == 0, (x2_con)

        ẋ(t) == dynamics(x(t), u(t), F(t))

        0.5 * ∫(x₁(t)^2 + x₂(t)^2 + u(t)^2) → min
    end

    function F(t)
        return sin(t * 2π / tf)
    end

    function dynamics(x, u, F)
        x1, x2, x3, x4 = x
        dx1 = x3
        dx2 = x4
        dx3 = -(k1 + k2) / m1 * x1 + k2 / m1 * x2 + 1 / m1 * F
        dx4 = k2 / m2 * x1 - k2 / m2 * x2 - c * (1 - u) / m2 * x4
        return [dx1, dx2, dx3, dx4]
    end

    # initial guess
    xinit = [0.1, 0.1, 0.1, 0.1]  # [x1, x2, x3, x4]
    uinit = [0.1]  # [u]
    init = (state=xinit, control=uinit)

    # DOCP and NLP
    docp = direct_transcription(ocp; init=init, grid_size=N, disc_method=:trapeze)
    nlp = model(docp)

    return docp, nlp
end
