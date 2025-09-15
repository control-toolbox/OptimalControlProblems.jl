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
function OptimalControlProblems.ducted_fan(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=steps_number_data(:ducted_fan),
    parameters::Union{Nothing, NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:ducted_fan, parameters)
    t0 = params[:t0]
    r = params[:r]
    J = params[:J]
    m = params[:m]
    mg = params[:mg]
    μ = params[:μ]
    α_l = params[:α_l]
    α_u = params[:α_u]
    u₁_l = params[:u₁_l]
    u₁_u = params[:u₁_u]
    u₂_l = params[:u₂_l]
    u₂_u = params[:u₂_u]
    tf_l = params[:tf_l]
    x_i = params[:x_i]
    x_f = params[:x_f]

    ocp = @def begin
        tf ∈ R, variable
        t ∈ [t0, tf], time
        x = (x₁, v₁, x₂, v₂, α, vα) ∈ R⁶, state
        u ∈ R², control

        # tf constraints
        tf ≥ tf_l, (tf_c)

        # state constraints
        α_l ≤ α(t) ≤ α_u, (α_c)

        # control constraints
        u₁_l ≤ u₁(t) ≤ u₁_u, (u₁_c)
        u₂_l ≤ u₂(t) ≤ u₂_u, (u₂_c)

        # initial constraints
        x₁(t0) == x_i[1], (x₁_i)
        v₁(t0) == x_i[2], (v₁_i)
        x₂(t0) == x_i[3], (x₂_i)
        v₂(t0) == x_i[4], (v₂_i)
        α(t0)  == x_i[5], (α_i)
        vα(t0) == x_i[6], (vα_i)

        # final constraints
        x₁(tf) == x_f[1], (x₁_f)
        v₁(tf) == x_f[2], (v₁_f)
        x₂(tf) == x_f[3], (x₂_f)
        v₂(tf) == x_f[4], (v₂_f)
        α(tf)  == x_f[5], (α_f)
        vα(tf) == x_f[6], (vα_f)

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
    varinit = [1.5]  # [tf] 
    init = (state=xinit, control=uinit, variable=varinit)

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
