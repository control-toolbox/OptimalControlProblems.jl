"""The **Aircraft Balanced Field Length Calculation** is a classic aeronautical engineering problem used to determine the minimum runway length required for a safe takeoff or a safe stop in the event of an engine failure.

Builds the `balanced_field` problem with the OptimalControl backend.

```julia-repl
julia> using OptimalControlProblems
julia> balanced_field(OptimalControlBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:balanced_field)` and `OptimalControlProblems.status(:balanced_field)`.
"""
function OptimalControlProblems.balanced_field(
    ::OptimalControlBackend; parameters::Union{Nothing,NamedTuple}=nothing
)
    OptimalControlProblems._warn_if_draft(:balanced_field, :OptimalControl)

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

    # model
    ocp = @def begin
        tf ∈ R, variable
        t ∈ [t0, tf], time
        x = (r, v, h, γ) ∈ R⁴, state
        α ∈ R, control

        r(t0) == r_t0
        v(t0) == v_t0
        h(t0) == h_t0
        γ(t0) == γ_t0

        h(tf) == h_tf
        γ(tf) == γ_tf

        0 ≤ α(t) ≤ α_max_ctrl # alpha
        tf ≥ 0.1
        h(t) ≥ 0
        v(t) ≥ 1.0

        ẋ(t) == dynamics(x(t), α(t), m, g, ρ, S, CD0, CL0, CL_max, α_max, T, b, K_nom, h_w)

        r(tf) → min
    end

    function dynamics(x, α, m, g, ρ, S, CD0, CL0, CL_max, α_max, T, b, K_nom, h_w)
        r, v, h, γ = x

        q = 0.5 * ρ * v^2
        CL = CL0 + (α / α_max) * (CL_max - CL0)
        L = q * S * CL

        h_eff = h + h_w
        term_h = 33.0 * abs(h_eff / b)^1.5
        K = K_nom * term_h / (1.0 + term_h)
        D = q * S * (CD0 + K * CL^2)

        rdot = v * cos(γ)
        vdot = (T * cos(α) - D) / m - g * sin(γ)
        hdot = v * sin(γ)
        γdot = (T * sin(α) + L) / (m * v) - (g * cos(γ)) / v

        return [rdot, vdot, hdot, γdot]
    end

    # initial guess
    tf_guess = 10.0
    xinit = [r_t0, v_t0 + 10.0, h_tf / 2.0, γ_tf]
    uinit = [0.1]
    vinit = [tf_guess]
    init = (state=xinit, control=uinit, variable=vinit)

    # discretise
    return OptimalControlProblems._problem(
        :balanced_field, :OptimalControl, ocp, init, params
    )
end
