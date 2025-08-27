"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** for the Van der Pol oscillator with a control input.  
The objective is to minimise a quadratic cost composed of the state and control effort over a fixed time horizon.  
The problem formulation can be found [here](https://github.com/control-toolbox/bocop/tree/main/bocop).

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `N::Int=500`: (Keyword) Number of discretisation points for the direct transcription grid.

# Returns

- `docp`: The direct optimal control problem object representing the Van der Pol problem.
- `nlp`: The corresponding nonlinear programming model obtained from the DOCP, suitable for numerical optimisation.

# Example

```julia-repl
julia> using OptimalControlProblems

julia> docp = OptimalControlProblems.vanderpol(OptimalControlBackend(); N=500);
```
"""
function OptimalControlProblems.vanderpol(::OptimalControlBackend; N::Int=steps_number_data(:vanderpol))

    # parameters
    tf = final_time_data(:vanderpol)
    ω = 1
    ε = 1

    # model
    ocp = @def begin
        t ∈ [0, tf], time
        x ∈ R², state
        u ∈ R, control

        x(0) == [1, 0]

        ẋ(t) == [x[2](t), ε * ω * (1 - x[1](t)^2) * x[2](t) - ω^2 * x[1](t) + u(t)]

        0.5∫(x[1](t)^2 + x[2](t)^2 + u(t)^2) → min
    end

    # initial guess
    xinit = [0.1, 0.1]  # [x1, x2]
    uinit = [0.1]  # [u]
    init = (state=xinit, control=uinit)

    # DOCP and NLP
    docp = direct_transcription(ocp; init=init, grid_size=N, disc_method=:trapeze)

    return docp
end
