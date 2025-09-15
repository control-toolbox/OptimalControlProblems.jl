"""
The Ducted Fan Problem:
    Implement the optimal control of a planar ducted fan.
    Instance taken from [GP2009].
    The problem is formulated as an OptimalControl model.
Ref: Graichen, K., & Petit, N. (2009). Incorporating a class of constraints into the dynamics of optimal control problems. Optimal Control Applications and Methods, 30(6), 537-561.
"""
function OptimalControlProblems.ducted_fan_s(
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
        ∂(x₁)(t) == v₁(t)
        ∂(v₁)(t) == (u₁(t) * cos(α(t)) - u₂(t) * sin(α(t))) / m
        ∂(x₂)(t) == v₂(t)
        ∂(v₂)(t) == (-mg + u₁(t) * sin(α(t)) + u₂(t) * cos(α(t))) / m
        ∂(α)(t) == vα(t)
        ∂(vα)(t) == r * u₁(t) / J

        # objective
        (1 / tf) * ∫(2 * u₁(t)^2 + u₂(t)^2) + (μ * tf) → min

    end

    # initial guess
    xinit = [0.1, 0.1, -0.1, 0.1, 0.1, 0.1]  # [x₁, v₁, x₂, v₂, α, vα]
    uinit = [0.1, 1]  # [u₁, u₂]
    varinit = [1.5]  # [tf]
    init = (state=xinit, control=uinit, variable=varinit)

    # DOCP
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