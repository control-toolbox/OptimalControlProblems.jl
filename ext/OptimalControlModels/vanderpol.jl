"""The **Van der Pol oscillator control problem** is a benchmark in nonlinear optimal control.

Builds the `vanderpol` problem with the OptimalControl backend.

```julia-repl
julia> using OptimalControlProblems
julia> vanderpol(OptimalControlBackend(); grid_size=100)
```

Default parameters and the modeling status are available through `metadata(:vanderpol)` and
`status(:vanderpol)`.
"""
function OptimalControlProblems.vanderpol(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=grid_size_data(:vanderpol),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)
    OptimalControlProblems._warn_if_draft(:vanderpol, :OptimalControl)

    # parameters
    params = parameters_data(:vanderpol, parameters)
    t0 = params[:t0]
    tf = params[:tf]
    ω = params[:ω]
    ε = params[:ε]
    x₁_t0 = params[:x₁_t0]
    x₂_t0 = params[:x₂_t0]

    # model
    ocp = @def begin
        t ∈ [t0, tf], time
        x ∈ R², state
        u ∈ R, control

        x(t0) == [x₁_t0, x₂_t0]

        ẋ(t) == [x[2](t), ε * ω * (1 - x[1](t)^2) * x[2](t) - ω^2 * x[1](t) + u(t)]

        0.5∫(x[1](t)^2 + x[2](t)^2 + u(t)^2) → min
    end

    # initial guess
    xinit = [0.1, 0.1]  # [x₁, x₂]
    uinit = [0.1]  # [u]
    init = (state=xinit, control=uinit)

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
