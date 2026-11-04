"""The **Jackson problem** is a classical benchmark in optimal control.

Builds the `jackson` problem with the OptimalControlScalarized backend.

```julia-repl
julia> using OptimalControlProblems
julia> jackson(OptimalControlScalarizedBackend())
```

Default parameters and the modeling status are available through
`problem_metadata(:jackson)` and `OptimalControlProblems.status(:jackson)`.
"""
function OptimalControlProblems.jackson(
    ::OptimalControlScalarizedBackend;
    parameters::Union{Nothing,NamedTuple}=nothing,
)
    OptimalControlProblems._warn_if_draft(:jackson, :OptimalControlScalarized)

    # parameters
    params = parameters_data(:jackson, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    k1 = params[:k1]
    k2 = params[:k2]
    k3 = params[:k3]
    a_l = params[:a_l]
    a_u = params[:a_u]
    b_l = params[:b_l]
    b_u = params[:b_u]
    x₃_l = params[:x₃_l]
    x₃_u = params[:x₃_u]
    u_l = params[:u_l]
    u_u = params[:u_u]
    a_t0 = params[:a_t0]
    b_t0 = params[:b_t0]
    x₃_t0 = params[:x₃_t0]

    # model
    ocp = @def begin
        t ∈ [t0, tf], time
        x = (a, b, x₃) ∈ R³, state
        u ∈ R, control

        x(t0) == [a_t0, b_t0, x₃_t0]

        [a_l, b_l, x₃_l] ≤ x(t) ≤ [a_u, b_u, x₃_u]
        u_l ≤ u(t) ≤ u_u

        ∂(x₁)(t) == -u(t) * (k1 * a(t) - k2 * b(t))
        ∂(x₂)(t) == u(t) * (k1 * a(t) - k2 * b(t)) - (1 - u(t)) * k3 * b(t)
        ∂(x₃)(t) == (1 - u(t)) * k3 * b(t)

        -x₃(tf) → min
    end

    # initial guess
    xinit = [0.1, 0.1, 0.1]  # [a, b, x₃]
    uinit = [0.1]  # [u]
    init = (state=xinit, control=uinit)

    return OptimalControlProblems._problem(:jackson, :OptimalControlScalarized, ocp, init, params)
end
