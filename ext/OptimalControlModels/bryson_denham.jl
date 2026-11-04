"""The Bryson-Denham problem is a constrained minimum-energy transfer problem.

Builds the `bryson_denham` problem with the OptimalControl backend.

```julia-repl
julia> using OptimalControlProblems
julia> bryson_denham(OptimalControlBackend(); grid_size=100)
```

Default parameters and the modeling status are available through `metadata(:bryson_denham)` and
`status(:bryson_denham)`.
"""

function OptimalControlProblems.bryson_denham(
    ::OptimalControlBackend,
    description::Symbol...;
    grid_size::Int=grid_size_data(:bryson_denham),
    parameters::Union{Nothing,NamedTuple}=nothing,
    kwargs...,
)

    OptimalControlProblems._warn_if_draft(:bryson_denham, :OptimalControl)
    params = parameters_data(:bryson_denham, parameters)
    t0, tf = params[:t0], params[:tf]
    x1_t0, x2_t0 = params[:x1_t0], params[:x2_t0]
    x1_tf, x2_tf = params[:x1_tf], params[:x2_tf]
    x1_max = params[:x1_max]

    ocp = @def begin
        t ∈ [t0, tf], time
        x ∈ R², state
        u ∈ R, control

        x(t0) == [x1_t0, x2_t0]
        x(tf) == [x1_tf, x2_tf]

        x₁(t) ≤ x1_max

        ẋ(t) == [x₂(t), u(t)]

        ∫(0.5 * u(t)^2) → min
    end

    init = (state=[0.0, 0.0], control=0.0)

    return direct_transcription(
        ocp,
        description...;
        lagrange_to_mayer=false,
        init=init,
        grid_size=grid_size,
        disc_method=:trapeze,
        kwargs...,
    )
end
