"""
The Truck Trailer Problem:
    We want to find the optimal trajectory of a truck with two trailers that starts horizontally aligned.
    The objective is to minimize the time taken to park the truck and the trailers aligned vertically at a given target location.
    The problem is formulated as an OptimalControl model.
"""
function OptimalControlProblems.truck_trailer(
    ::OptimalControlBackend;
    data::Array{Float64,2}=[0.4 0.1 0.2; 1.1 0.2 0.2; 0.8 0.1 0.2],
    nh::Int=200,
)

    # parameters
    if size(data) != (3, 3)
        error("The input data matrix must be 3x3.")
    end
    L0 = data[1, 1]
    M0 = data[1, 2]
    W0 = data[1, 3]
    L1 = data[2, 1]
    M1 = data[2, 2]
    W1 = data[2, 3]
    L2 = data[3, 1]
    M2 = data[3, 2]
    W2 = data[3, 3]
    speedf = 1
    x2_t0 = 0
    y2_t0 = 0
    θ2_t0 = 0
    θ1_t0 = 0
    θ0_t0 = 0
    x2_tf = 0
    y2_tf = -2
    θ2_tf = π / 2
    θ1_tf = π / 2
    θ0_tf = π / 2

    # model
    ocp = @def begin

        tf ∈ R, variable
        t ∈ [0, tf], time
        x = (x2, y2, θ0, θ1, θ2, v0, δ0) ∈ R⁷, state
        u = (dv0, dδ0) ∈ R², control

        # auxiliary variables
        β01 = θ0 - θ1
        β12 = θ1 - θ2
        # x1 = x2 + L2 * cos(θ2) + M1 * cos(θ1)
        # y1 = y2 + L2 * sin(θ2) + M1 * sin(θ1)
        # x0 = x1 + L1 * cos(θ1) + M0 * cos(θ0)
        # y0 = y1 + L1 * sin(θ1) + M0 * sin(θ0)

        # final time constraints
        1 ≤ tf ≤ 1000
        
        # state constraints
        -π / 2 ≤ θ0(t) ≤ π / 2, (θ0_con)
        -π / 2 ≤ θ1(t) ≤ π / 2, (θ1_con)
        -0.2 * speedf ≤ v0(t) ≤ 0.2 * speedf, (v0_con)
        -π / 6 ≤ δ0(t) ≤ π / 6, (δ0_con)

        # control constraints
        -1 ≤ dv0(t) ≤ 1, (v0_dot_con)
        -π / 10 ≤ dδ0(t) ≤ π / 10, (δ0_dot_con)

        # path constraints
        -π / 2 ≤ β01(t) ≤ π / 2, (β01_con)
        -π / 2 ≤ β12(t) ≤ π / 2, (β12_con)

        # initial conditions
        x2(0) == x2_t0, (x2_t0_con)
        y2(0) == y2_t0, (y2_t0_con)
        θ0(0) == θ0_t0, (θ0_t0_con)
        θ1(0) == θ1_t0, (θ1_t0_con)
        θ2(0) == θ2_t0, (θ2_t0_con)

        # final conditions
        x2(tf) == x2_tf, (x2_tf_con)
        y2(tf) == y2_tf, (y2_tf_con)
        θ2(tf) == θ2_tf, (θ2_tf_con)
        β01(tf) == θ0_tf - θ1_tf, (β01_tf_con)
        β12(tf) == θ1_tf - θ2_tf, (β12_tf_con)

        # dynamics
        ẋ(t) == dynamics(x(t), u(t))

        # objective
        tf + ∫( β01(t)^2 + β12(t)^2 ) → min

    end

    function dynamics(x, u)
        x2, y2, θ0, θ1, θ2, v0, δ0 = x
        dv0, dδ0 = u

        β01 = θ0 - θ1
        β12 = θ1 - θ2

        dθ0 = v0 / L0 * tan(δ0)
        dθ1 = v0 / L1 * sin(β01) - M0 / L1 * cos(β01) * dθ0

        v1 = v0 * cos(β01) + M0 * sin(β01) * dθ0
        dθ2 = v1 / L2 * sin(β12) - M1 / L2 * cos(β12) * dθ1
        v2 = v1 * cos(β12) + M1 * sin(β12) * dθ1

        dx2 = v2 * cos(θ2)
        dy2 = v2 * sin(θ2)

        return [dx2, dy2, dθ0, dθ1, dθ2, dv0, dδ0]
    end

    # initial guess
    xinit = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]  # [x2, y2, θ0, θ1, θ2, v0, δ0]
    uinit = [0.1, 0.1]  # [dv0, dδ0]
    varinit = [10]  # [tf]
    init = (state=xinit, control=uinit, variable=varinit)

    # DOCP and NLP
    docp = direct_transcription(ocp; init=init, grid_size=nh)
    nlp = model(docp)

    return docp, nlp
end
