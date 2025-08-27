"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** for a planar ducted fan system.  
The function defines state and control variables, system dynamics, boundary conditions, and a cost functional combining control effort and final time.  
It returns both a discretised direct optimal control problem (DOCP) and the corresponding nonlinear programming (NLP) model.

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `N::Int=250`: (Keyword) Number of discretisation points for the direct transcription grid.

# Returns

- `docp`: The direct optimal control problem object representing the planar ducted fan.
- `nlp`: The corresponding nonlinear programming model obtained from the DOCP, suitable for numerical optimisation.

# Example

```julia-repl
julia> using OptimalControlProblems

julia> docp = OptimalControlProblems.ducted_fan(OptimalControlBackend(); N=250);
```

# References

- Graichen, K., & Petit, N. (2009). Incorporating a class of constraints into the dynamics of optimal control problems. *Optimal Control Applications and Methods*, 30(6), 537-561. [GP2009]
- Problem instance follows OptimalControl formulation for ducted fan trajectory optimisation.
"""
function OptimalControlProblems.ducted_fan(::OptimalControlBackend; N::Int=steps_number_data(:ducted_fan))

    # parameters
    r = 0.2         # [m]
    J = 0.05        # [kg.m2]
    m = 2.2         # [kg]
    mg = 4          # [N]
    μ = 1000

    ocp = @def begin
        tf ∈ R, variable
        t ∈ [0, tf], time
        x = (x₁, v₁, x₂, v₂, α, vα) ∈ R⁶, state
        u ∈ R², control

        # tf constraints
        tf ≥ 0.1, (tf_con)

        # state constraints
        -deg2rad(30) ≤ α(t) ≤ deg2rad(30), (α_con)

        # control constraints
        -5 ≤ u₁(t) ≤ 5, (u₁_con)
        0 ≤ u₂(t) ≤ 17, (u₂_con)

        # initial constraints
        x₁(0) == 0, (x₁_i)
        v₁(0) == 0, (v₁_i)
        x₂(0) == 0, (x₂_i)
        v₂(0) == 0, (v₂_i)
        α(0) == 0, (α_i)
        vα(0) == 0, (vα_i)

        # final constraints
        x₁(tf) == 1, (x₁_f)
        v₁(tf) == 0, (v₁_f)
        x₂(tf) == 0, (x₂_f)
        v₂(tf) == 0, (v₂_f)
        α(tf) == 0, (α_f)
        vα(tf) == 0, (vα_f)

        # dynamics
        ẋ(t) == dynamics(v₁(t), v₂(t), α(t), vα(t), u₁(t), u₂(t))

        # objective
        (1 / tf) * ∫(2 * u₁(t)^2 + u₂(t)^2) + (μ * tf) → min
    end

    function dynamics(v₁, v₂, α, vα, u₁, u₂)
        dx₁ = v₁
        dv₁ = (u₁ * cos(α) - u₂ * sin(α)) / m
        dx₂ = v₂
        dv₂ = (-mg + u₁ * sin(α) + u₂ * cos(α)) / m
        dα = vα
        dvα = r * u₁ / J

        return [dx₁, dv₁, dx₂, dv₂, dα, dvα]
    end

    # initial guess
    xinit = [0.1, 0.1, -0.1, 0.1, 0.1, 0.1]  # [x₁, v₁, x₂, v₂, α, vα]
    uinit = [0.1, 1]  # [u₁, u₂]
    varinit = [1]  # [tf] 
    init = (state=xinit, control=uinit, variable=varinit)

    # DOCP and NLP
    docp = direct_transcription(ocp; init=init, grid_size=N, disc_method=:trapeze)

    return docp
end
