"""
The Robbins Problem:
    The problem is formulated as an OptimalControl model and can be found [here](https://github.com/control-toolbox/bocop/tree/main/bocop)
"""
function OptimalControlProblems.robbins(::OptimalControlBackend; nh::Int=500)
    
    # parameters
    α = 3
    β = 0
    γ = 0.5
    tf = 10

    # model
    ocp = @def begin
        
        t ∈ [0, tf], time
        x ∈ R³, state
        u ∈ R, control

        0 ≤ x[1](t) ≤ Inf

        x(0) == [1, -2, 0]
        x(tf) == [0, 0, 0]

        ẋ(t) == [x[2](t), x[3](t), u(t)]

        ∫(α * x[1](t) + β * x[1](t)^2 + γ * u(t)^2) → min

    end

    # initial guess
    xinit = [0.1, 0.1, 0.1]  # [x1, x2, x3]
    uinit = [0.1]  # [u]
    init = (state=xinit, control=uinit)

    # DOCP and NLP
    docp = direct_transcription(ocp; init=init, grid_size=nh)
    nlp = model(docp)

    return docp, nlp
end
