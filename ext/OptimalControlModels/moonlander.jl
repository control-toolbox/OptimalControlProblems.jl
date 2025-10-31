"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** for the Moonlander benchmark model.  
The function defines the state and control variables, system dynamics, bounds, initial and final conditions, and the cost functional, which minimises the landing time of a moonlander to a specified target.  
It returns both a discretised direct optimal control problem (DOCP) and the corresponding nonlinear programming (NLP) model.

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `grid_size::Int=500`: (Keyword) Number of discretisation points for the direct transcription grid.

# Returns

- `docp`: The direct optimal control problem object representing the Moonlander problem.
- `nlp`: The corresponding nonlinear programming model obtained from the DOCP, suitable for numerical optimisation.

# Example

```julia-repl
julia> using OptimalControlProblems

julia> docp = OptimalControlProblems.moonlander(OptimalControlBackend(); N=500);
```
"""
function OptimalControlProblems.moonlander(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=grid_size_data(:moonlander),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)

    # parameters
    params = parameters_data(:moonlander, parameters)
    t0 = params[:t0]
    m = params[:m]
    g = params[:g]
    I = params[:I]
    D = params[:D]
    max_thrust = params[:max_thrust]
    tf_l = params[:tf_l]
    tf_u = params[:tf_u]
    F₁_l = params[:F₁_l]
    F₁_u = max_thrust
    F₂_l = params[:F₂_l]
    F₂_u = max_thrust
    tf_l = params[:tf_l]
    tf_u = params[:tf_u]
    F₁_l = params[:F₁_l]
    F₂_l = params[:F₂_l]
    p₁_t0 = params[:p₁_t0]
    p₂_t0 = params[:p₂_t0]
    dp₁_t0 = params[:dp₁_t0]
    dp₂_t0 = params[:dp₂_t0]
    θ_t0 = params[:θ_t0]
    dθ_t0 = params[:dθ_t0]
    p₁_tf = params[:p₁_tf]
    p₂_tf = params[:p₂_tf]
    dp₁_tf = params[:dp₁_tf]
    dp₂_tf = params[:dp₂_tf]

    # define the problem
    ocp = @def begin

        # state, control and final time variables, and time
        tf ∈ R, variable
        t ∈ [t0, tf], time
        x = (p₁, p₂, dp₁, dp₂, θ, dθ) ∈ R⁶, state
        u = (F₁, F₂) ∈ R², control

        # final time constraint
        tf_l ≤ tf ≤ tf_u

        # control constraints
        F₁_l ≤ F₁(t) ≤ F₁_u, (F₁_c)
        F₂_l ≤ F₂(t) ≤ F₂_u, (F₂_c)

        # initial conditions
        p₁(t0) == p₁_t0, (p₁_t0)
        p₂(t0) == p₂_t0, (p₂_t0)
        dp₁(t0) == dp₁_t0, (dp₁_t0)
        dp₂(t0) == dp₂_t0, (dp₂_t0)
        θ(t0) == θ_t0, (θ_t0)
        dθ(t0) == dθ_t0, (dθ_t0)

        # final conditions
        p₁(tf) == p₁_tf, (p₁_tf)
        p₂(tf) == p₂_tf, (p₂_tf)
        dp₁(tf) == dp₁_tf, (dp₁_tf)
        dp₂(tf) == dp₂_tf, (dp₂_tf)

        # dynamics
        ẋ(t) == dynamics(x(t), u(t))

        # objective
        tf → min
    end

    # dynamics
    function dynamics(x, u)
        p₁, p₂, dp₁, dp₂, θ, dθ = x
        F₁, F₂ = u

        F_r = [
            cos(θ) -sin(θ) p₁
            sin(θ) cos(θ) p₂
            0 0 1
        ]
        F_tot = (F_r * [0; F₁ + F₂; 0])[1:2]
        ddp₁ = (1 / m) * F_tot[1]
        ddp₂ = (1 / m) * F_tot[2] - g
        ddθ = (1 / I) * (D / 2) * (F₂ - F₁)

        return [dp₁, dp₂, ddp₁, ddp₂, dθ, ddθ]
    end

    # initial guess
    xinit = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]  # [p₁, p₂, dp₁, dp₂, θ, dθ]
    uinit = [5.0, 5.0]  # [F₁, F₂] 
    varinit = [0.5]  # [tf] 
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
