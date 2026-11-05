"""The **robot arm motion problem** is a benchmark in time-optimal control.

Builds the `robot` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> robot(JuMPBackend(); grid_size=100)
```

Default parameters and the modeling status are available through
`problem_metadata(:robot)` and `OptimalControlProblems.status(:robot)`.
"""
function OptimalControlProblems.robot(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:robot),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)
    OptimalControlProblems._warn_if_draft(:robot, :JuMP)

    # parameters
    params = parameters_data(:robot, parameters)
    t0 = params[:t0]

    # total length of arm
    L = params[:L]

    # Upper bounds on the controls
    uρ_l = params[:uρ_l]
    uρ_u = params[:uρ_u]
    uθ_l = params[:uθ_l]
    uθ_u = params[:uθ_u]
    uϕ_l = params[:uϕ_l]
    uϕ_u = params[:uϕ_u]

    # Initial positions of the length and the angles for the robot arm
    ρ_t0 = params[:ρ_t0]
    θ_t0 = params[:θ_t0]
    ϕ_t0 = params[:ϕ_t0]
    dρ_t0 = params[:dρ_t0]
    dθ_t0 = params[:dθ_t0]
    dϕ_t0 = params[:dϕ_t0]

    # Final positions
    ρ_tf = params[:ρ_tf]
    θ_tf = params[:θ_tf]
    ϕ_tf = params[:ϕ_tf]
    dρ_tf = params[:dρ_tf]
    dθ_tf = params[:dθ_tf]
    dϕ_tf = params[:dϕ_tf]

    #
    ρ_l = params[:ρ_l]
    ρ_u = L
    θ_l = params[:θ_l]
    θ_u = params[:θ_u]
    ϕ_l = params[:ϕ_l]
    ϕ_u = params[:ϕ_u]
    tf_l = params[:tf_l]

    # initial guess: linear interpolation between the boundary conditions, ϕ kept away
    # from zero (division by sin(ϕ)²)
    interpolate(a, b) = [a + (k / grid_size) * (b - a) for k in 0:grid_size]
    ϕ_start = [abs(ϕ) < 1e-6 ? 1e-6 : ϕ for ϕ in interpolate(ϕ_t0, ϕ_tf)]

    function dynamics(x, u)
        ρ, dρ, θ, dθ, ϕ, dϕ = x
        uρ, uθ, uϕ = u
        I_θ = ((L - ρ)^3 + ρ^3) * ((sin(ϕ))^2 + 1e-9)
        I_ϕ = (L - ρ)^3 + ρ^3
        return [dρ, uρ / L, dθ, 3 * uθ / I_θ, dϕ, 3 * uϕ / I_ϕ]
    end

    data = ProblemData(;
        state=[
            Component("ρ"; lower=ρ_l, upper=ρ_u, start=interpolate(ρ_t0, ρ_tf)),
            Component("dρ"; start=0.0),
            Component("θ"; lower=θ_l, upper=θ_u, start=interpolate(θ_t0, θ_tf)),
            Component("dθ"; start=0.0),
            Component("ϕ"; lower=ϕ_l, upper=ϕ_u, start=ϕ_start),
            Component("dϕ"; start=0.0),
        ],
        control=[
            Component("uρ"; lower=uρ_l, upper=uρ_u, start=0.0),
            Component("uθ"; lower=uθ_l, upper=uθ_u, start=0.0),
            Component("uϕ"; lower=uϕ_l, upper=uϕ_u, start=0.0),
        ],
        variable=[Component("tf"; lower=tf_l, start=9.1)],
        costate=["∂ρ", "∂dρ", "∂θ", "∂dθ", "∂ϕ", "∂dϕ"],
        times=v -> (t0, v[1]),
        dynamics=(x, u, v, t) -> dynamics(x, u),
        cost=(I, x0, xf, v) -> v[1],
        boundary=[
            (ρ_t0, (x0, xf, v) -> x0[1], ρ_t0),
            (θ_t0, (x0, xf, v) -> x0[3], θ_t0),
            (ϕ_t0, (x0, xf, v) -> x0[5], ϕ_t0),
            (dρ_t0, (x0, xf, v) -> x0[2], dρ_t0),
            (dθ_t0, (x0, xf, v) -> x0[4], dθ_t0),
            (dϕ_t0, (x0, xf, v) -> x0[6], dϕ_t0),
            (ρ_tf, (x0, xf, v) -> xf[1], ρ_tf),
            (θ_tf, (x0, xf, v) -> xf[3], θ_tf),
            (ϕ_tf, (x0, xf, v) -> xf[5], ϕ_tf),
            (dρ_tf, (x0, xf, v) -> xf[2], dρ_tf),
            (dθ_tf, (x0, xf, v) -> xf[4], dθ_tf),
            (dϕ_tf, (x0, xf, v) -> xf[6], dϕ_tf),
        ],
    )
    return build_trapeze(data, grid_size, args...; kwargs...)
end
