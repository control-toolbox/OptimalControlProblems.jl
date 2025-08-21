"""
$(TYPEDSIGNATURES)

Constructs an **OptimalControl problem** for the Quadrotor benchmark model.  
This function defines the state and control variables, system dynamics, bounds, initial and final conditions, and the cost functional, which minimises the final time to reach a target position while including small penalties on control inputs.  
It returns both a discretised direct optimal control problem (DOCP) and the corresponding nonlinear programming (NLP) model.

# Arguments

- `::OptimalControlBackend`: Placeholder type specifying the OptimalControl backend or solver interface.
- `N::Int=50`: (Keyword) Number of discretisation points for the direct transcription grid.

# Returns

- `docp`: The direct optimal control problem object representing the Quadrotor problem.
- `nlp`: The corresponding nonlinear programming model obtained from the DOCP, suitable for numerical optimisation.

# Example

```julia-repl
julia> using OptimalControlProblems

julia> docp, nlp = OptimalControlProblems.quadrotor(OptimalControlBackend(); N=50);

julia> docp
DOCP object with 50 discretisation points

julia> nlp
NLP model corresponding to the Quadrotor problem
```
"""
function OptimalControlProblems.quadrotor(::OptimalControlBackend; N::Int=50)

    # parameters
    g = 9.81
    atmin = 0
    atmax = 9.18 * 5
    tiltmax = 1.1 / 2
    dtiltmax = 6 / 2
    p0 = [0, 0, 2.5]
    v0 = [0, 0, 0]
    u0 = [9.81, 0, 0, 0]
    pf = [0.01, 5, 2.5]
    vf = [0, 0, 0]

    ocp = @def begin
        tf ∈ R, variable
        t ∈ [0, tf], time
        x = (p₁, p₂, p₃, v₁, v₂, v₃, ϕ, θ) ∈ R⁸, state
        u = (at, dϕ, dθ, ψ) ∈ R⁴, control

        # state constraints
        tf ≥ 0.1, (tf_con)
        -π / 2 ≤ ϕ(t) ≤ π / 2, (ϕ_con)
        -π / 2 ≤ θ(t) ≤ π / 2, (θ_con)

        # control constraints
        atmin ≤ at(t) ≤ atmax, (at_con)
        -dtiltmax ≤ dϕ(t) ≤ dtiltmax, (ϕdot_con)
        -dtiltmax ≤ dθ(t) ≤ dtiltmax, (θdot_con)

        # path constraints
        cos(θ(t)) * cos(ϕ(t)) ≥ cos(tiltmax), (tiltmax_con)

        # initial constraints
        p₁(0) == p0[1], (p₁_i)
        p₂(0) == p0[2], (p₂_i)
        p₃(0) == p0[3], (p₃_i)
        v₁(0) == v0[1], (v₁_i)
        v₂(0) == v0[2], (v₂_i)
        v₃(0) == v0[3], (v₃_i)
        ϕ(0) == u0[2], (ϕ_i)
        θ(0) == u0[3], (θ_i)

        # final constraints
        p₁(tf) == pf[1], (p₁_f)
        p₂(tf) == pf[2], (p₂_f)
        p₃(tf) == pf[3], (p₃_f)
        v₁(tf) == vf[1], (v₁_f)
        v₂(tf) == vf[2], (v₂_f)
        v₃(tf) == vf[3], (v₃_f)

        # dynamics
        ẋ(t) == dynamics(x(t), u(t))

        # objective  
        tf + ∫(1e-8 * (at(t)^2 + ϕ(t)^2 + θ(t)^2 + ψ(t)^2) + 1e2 * (ψ(t) - u0[3])^2) → min
    end

    function dynamics(x, u)
        p₁, p₂, p₃, v₁, v₂, v₃, ϕ, θ = x
        at, dϕ, dθ, ψ = u

        cr = cos(ϕ)
        sr = sin(ϕ)
        cp = cos(θ)
        sp = sin(θ)
        cy = cos(ψ)
        sy = sin(ψ)
        R = [
            (cy * cp) (cy * sp * sr - sy * cr) (cy * sp * cr + sy * sr)
            (sy * cp) (sy * sp * sr + cy * cr) (sy * sp * cr - cy * sr)
            (-sp) (cp * sr) (cp * cr)
        ]
        at_ = R * [0; 0; at]
        g_ = [0; 0; -g]
        a = at_ + g_

        return [v₁, v₂, v₃, a[1], a[2], a[3], dϕ, dθ]
    end

    # initial guess
    xinit = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]  # [p₁, p₂, p₃, v₁, v₂, v₃, ϕ, θ]
    uinit = [10, 0.1, 0.1, 0.1]  # [at, dϕ, dθ, ψ] 
    varinit = [1]  # [tf]
    init = (state=xinit, control=uinit, variable=varinit)

    # DOCP and NLP
    docp = direct_transcription(ocp; init=init, grid_size=N, disc_method=:trapeze)
    nlp = model(docp)

    return docp, nlp
end
