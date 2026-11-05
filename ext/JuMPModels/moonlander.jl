"""The **Moonlander optimal control problem** is a benchmark in constrained optimal control.

Builds the `moonlander` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> moonlander(JuMPBackend(); grid_size=100)
```

Default parameters and the modeling status are available through
`problem_metadata(:moonlander)` and `OptimalControlProblems.status(:moonlander)`.
"""
function OptimalControlProblems.moonlander(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:moonlander),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)
    OptimalControlProblems._warn_if_draft(:moonlander, :JuMP)

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

    function dynamics(x, u)
        p₁, p₂, dp₁, dp₂, θ, dθ = x
        F₁, F₂ = u
        F_r = [
            cos(θ) -sin(θ) p₁
            sin(θ) cos(θ) p₂
            0 0 1
        ]
        F_tot = (F_r * [0; F₁ + F₂; 0])[1:2]
        return [
            dp₁,
            dp₂,
            (1 / m) * F_tot[1],
            (1 / m) * F_tot[2] - g,
            dθ,
            (1 / I) * (D / 2) * (F₂ - F₁),
        ]
    end

    data = ProblemData(;
        state=[
            Component("p₁"; start=0.1),
            Component("p₂"; start=0.1),
            Component("dp₁"; start=0.1),
            Component("dp₂"; start=0.1),
            Component("θ"; start=0.1),
            Component("dθ"; start=0.1),
        ],
        control=[
            Component("F₁"; lower=F₁_l, upper=F₁_u, start=5.0),
            Component("F₂"; lower=F₂_l, upper=F₂_u, start=5.0),
        ],
        variable=[Component("tf"; lower=tf_l, upper=tf_u, start=0.5)],
        costate=["∂p₁", "∂p₂", "∂dp₁", "∂dp₂", "∂θ", "∂dθ"],
        times=v -> (t0, v[1]),
        dynamics=(x, u, v, t) -> dynamics(x, u),
        cost=(integral, x0, xf, v) -> v[1],
        boundary=[
            (p₁_t0, (x0, xf, v) -> x0[1], p₁_t0),
            (p₂_t0, (x0, xf, v) -> x0[2], p₂_t0),
            (dp₁_t0, (x0, xf, v) -> x0[3], dp₁_t0),
            (dp₂_t0, (x0, xf, v) -> x0[4], dp₂_t0),
            (θ_t0, (x0, xf, v) -> x0[5], θ_t0),
            (dθ_t0, (x0, xf, v) -> x0[6], dθ_t0),
            (p₁_tf, (x0, xf, v) -> xf[1], p₁_tf),
            (p₂_tf, (x0, xf, v) -> xf[2], p₂_tf),
            (dp₁_tf, (x0, xf, v) -> xf[3], dp₁_tf),
            (dp₂_tf, (x0, xf, v) -> xf[4], dp₂_tf),
        ],
    )
    return build_trapeze(data, grid_size, args...; kwargs...)
end
