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
    N::Int=steps_number_data(:ducted_fan),
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

    ocp = @def begin
        tf ∈ R, variable
        t ∈ [t0, tf], time
        x = (x₁, v₁, x₂, v₂, α, vα) ∈ R⁶, state
        u ∈ R², control

        # tf constraints
        tf ≥ 0.1, (tf_c)

        # state constraints
        -deg2rad(30) ≤ α(t) ≤ deg2rad(30), (α_c)

        # control constraints
        -5 ≤ u₁(t) ≤ 5, (u₁_c)
        0 ≤ u₂(t) ≤ 17, (u₂_c)

        # initial constraints
        x₁(t0) == 0, (x₁_i)
        v₁(t0) == 0, (v₁_i)
        x₂(t0) == 0, (x₂_i)
        v₂(t0) == 0, (v₂_i)
        α(t0) == 0, (α_i)
        vα(t0) == 0, (vα_i)

        # final constraints
        x₁(tf) == 1, (x₁_f)
        v₁(tf) == 0, (v₁_f)
        x₂(tf) == 0, (x₂_f)
        v₂(tf) == 0, (v₂_f)
        α(tf) == 0, (α_f)
        vα(tf) == 0, (vα_f)

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
        grid_size=N,
        disc_method=:trapeze,
        kwargs...,
    )

    return docp
end