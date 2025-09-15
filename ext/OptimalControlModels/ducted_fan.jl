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
    grid_size::Int=grid_size_data(:ducted_fan),
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
    x₁_t0 = params[:x₁_t0]
    v₁_t0 = params[:v₁_t0]
    x₂_t0 = params[:x₂_t0]
    v₂_t0 = params[:v₂_t0]
    α_t0 = params[:α_t0]
    vα_t0 = params[:vα_t0]
    x₁_tf = params[:x₁_tf]
    v₁_tf = params[:v₁_tf]
    x₂_tf = params[:x₂_tf]
    v₂_tf = params[:v₂_tf]
    α_tf = params[:α_tf]
    vα_tf = params[:vα_tf]

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
        x₁(t0) == x₁_t0, (x₁_t0)
        v₁(t0) == v₁_t0, (v₁_t0)
        x₂(t0) == x₂_t0, (x₂_t0)
        v₂(t0) == v₂_t0, (v₂_t0)
        α(t0)  == α_t0, (α_t0)
        vα(t0) == vα_t0, (vα_t0)

        # final constraints
        x₁(tf) == x₁_tf, (x₁_tf)
        v₁(tf) == v₁_tf, (v₁_tf)
        x₂(tf) == x₂_tf, (x₂_tf)
        v₂(tf) == v₂_tf, (v₂_tf)
        α(tf)  == α_tf , (α_tf )
        vα(tf) == vα_tf, (vα_tf)

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
