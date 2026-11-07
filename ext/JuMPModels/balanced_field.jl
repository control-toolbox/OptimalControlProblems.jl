"""The **Aircraft Balanced Field Length Calculation** is a classic aeronautical engineering problem used to determine the minimum runway length required for a safe takeoff or a safe stop in the event of an engine failure.

Builds the `balanced_field` problem with the JuMP backend.

```julia-repl
julia> using OptimalControlProblems
julia> balanced_field(JuMPBackend(); grid_size=100)
```

The keyword `scheme` chooses the discretization: `:trapeze` (default), `:euler`,
`:euler_implicit` or `:midpoint`, as in `OptimalControl.Collocation`.
The keyword `ordering` chooses the order of the variables and constraints:
`:interleaved` (default, as in CTDirect) or `:blocks` (grouped by kind).

Default parameters and the modeling status are available through
`problem_metadata(:balanced_field)` and `OptimalControlProblems.status(:balanced_field)`.
"""
function OptimalControlProblems.balanced_field(
    ::JuMPBackend,
    args...;
    grid_size::Int=grid_size_data(:balanced_field),
    scheme::Symbol=:trapeze,
    ordering::Symbol=:interleaved,
    parameters::Union{Nothing,NamedTuple}=nothing,
    init=nothing,
    kwargs...,
)
    OptimalControlProblems._warn_if_draft(:balanced_field, :JuMP)

    # parameters
    params = parameters_data(:balanced_field, parameters)
    t0 = params[:t0]
    m = params[:m]
    g = params[:g]
    ρ = params[:ρ]
    S = params[:S]
    CD0 = params[:CD0]
    AR = params[:AR]
    e = params[:e]
    CL0 = params[:CL0]
    CL_max = params[:CL_max]
    h_w = params[:h_w]
    span = params[:span]
    α_max = params[:α_max]
    T = params[:T]

    r_t0 = params[:r_t0]
    v_t0 = params[:v_t0]
    h_t0 = params[:h_t0]
    γ_t0 = params[:γ_t0]
    h_tf = params[:h_tf]
    γ_tf = params[:γ_tf]

    α_min = params[:α_min]
    α_max_ctrl = params[:α_max_ctrl]

    # Constants
    b = span / 2.0
    K_nom = 1.0 / (pi * AR * e)

    function dynamics(x, u)
        r, v, h, γ = x
        α = u[1]
        q = 0.5 * ρ * v^2
        CL = CL0 + (α / α_max) * (CL_max - CL0)
        L = q * S * CL
        h_eff = h + h_w
        term_h = 33.0 * abs(h_eff / b)^1.5
        K = K_nom * term_h / (1.0 + term_h)
        D = q * S * (CD0 + K * CL^2)
        return [
            v * cos(γ),
            (T * cos(α) - D) / m - g * sin(γ),
            v * sin(γ),
            (T * sin(α) + L) / (m * v) - (g * cos(γ)) / v,
        ]
    end

    data = ProblemData(;
        state=[
            Component("r"),
            Component("v"; lower=1.0),
            Component("h"; lower=0.0),
            Component("γ"),
        ],
        control=[Component("α"; lower=0, upper=α_max_ctrl)],
        variable=[Component("tf"; lower=0.1)],
        costate=["∂r", "∂v", "∂h", "∂γ"],
        times=v -> (t0, v[1]),
        dynamics=(x, u, v, t) -> dynamics(x, u),
        cost=(I, x0, xf, v) -> xf[1],
        boundary=[
            (r_t0, (x0, xf, v) -> x0[1], r_t0),
            (v_t0, (x0, xf, v) -> x0[2], v_t0),
            (h_t0, (x0, xf, v) -> x0[3], h_t0),
            (γ_t0, (x0, xf, v) -> x0[4], γ_t0),
            (h_tf, (x0, xf, v) -> xf[3], h_tf),
            (γ_tf, (x0, xf, v) -> xf[4], γ_tf),
        ],
    )
    # initial guess: from the metadata (#377), unless given
    init = OptimalControlProblems._initial_guess(:balanced_field, params, init)
    return build(data, grid_size, scheme, ordering, args...; init=init, kwargs...)
end
