isdefined(Main, :SolveModelingToolkit) ||
    include(joinpath(@__DIR__, "..", "..", "common", "solve_modelingtoolkit.jl"))

module TestMethodsModelingToolkit

using Test: Test
using ModelingToolkit: ModelingToolkit as MTK
using OptimalControlProblems: OptimalControlProblems as OCP

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

function test_methods()
    Test.@testset "ModelingToolkit solve helpers" verbose = VERBOSE showtiming = SHOWTIMING begin
        backend = OCP.ModelingToolkitBackend()
        pb = OCP.bryson_denham(backend)
        sys = OCP.model(pb)

        # 1. tspan: from the parameters of the problem, the same as the domain of the integral
        tspan = Main.SolveModelingToolkit.tspan(pb)
        Test.@test tspan == (0.0, 1.0)
        domain = MTK.SymbolicUtils.operation(MTK.get_costs(sys)[1]).domain.domain
        Test.@test (Float64(domain.left), Float64(domain.right)) == tspan

        # 1b. a free final time: the system is in the reduced time, tf is a variable to optimize
        free = OCP.mountain_car(backend)
        Test.@test Main.SolveModelingToolkit.free_final_time(OCP.model(free))
        Test.@test !Main.SolveModelingToolkit.free_final_time(sys)
        Test.@test OCP.variable_components(OCP.model(free)) == ["tf"]
        Test.@test Main.SolveModelingToolkit.times(free) == (0.0, nothing)
        Test.@test Main.SolveModelingToolkit.times(pb) == (0.0, 1.0)
        Test.@test Main.SolveModelingToolkit.tspan(free) == (0.0, 1.0)
        # started from the guess of the variable
        started = Main.SolveModelingToolkit.prepare(free; grid_size=6).dynamic
        Test.@test Main.SolveModelingToolkit.variable_values(started, "tf") ==
            [OCP.initial_guess(free).variable]

        # 2. _is_path_constraint
        iv = MTK.independent_variable(sys)
        cons = MTK.get_constraints(sys)
        path_cons = filter(c -> Main.SolveModelingToolkit._is_path_constraint(c, iv), cons)
        bc_cons = filter(c -> !Main.SolveModelingToolkit._is_path_constraint(c, iv), cons)
        Test.@test length(path_cons) == 1
        Test.@test length(bc_cons) == 2

        # 3. _discretize_path_constraints
        grid = range(0.0, 1.0, length=5)
        sys_disc = Main.SolveModelingToolkit._discretize_path_constraints(sys, grid)
        disc_cons = MTK.get_constraints(sys_disc)
        Test.@test length(disc_cons) == length(bc_cons) + length(grid)

        # 4. solver_options
        opts = Main.SolveModelingToolkit.solver_options(:ipopt; max_iter=10)
        Test.@test opts.max_iter == 10
        Test.@test opts.tol == Main.TestConfigs.TOL
        Test.@test_throws ArgumentError Main.SolveModelingToolkit.solver_options(:madnlp)

        # 5. settings and scheme
        cfg = (shooting=:single, solver=:ipopt)
        stg = Main.SolveModelingToolkit.settings(cfg; max_iter=5)
        Test.@test stg.modeler === :corleone
        Test.@test stg.solver === :ipopt
        Test.@test stg.options.max_iter == 5
        Test.@test Main.SolveModelingToolkit.scheme(cfg) === :shooting_single

        # 6. control_start: the initial guess of the problem (#377)
        S = Main.SolveModelingToolkit
        Test.@test S.control_start((control=0.5,), 0.0, 2) == [0.5, 0.5]
        Test.@test S.control_start((control=[1, 2],), 0.0, 2) == [1.0, 2.0]
        Test.@test S.control_start((control=t -> 3t,), 1.0, 1) == [3.0]
        Test.@test S.control_start((state=[0.0, 0.0],), 0.0, 1) == [0.0]
        Test.@test_throws DimensionMismatch S.control_start((control=[1, 2],), 0.0, 1)
        # the states of free initial value start from the state guess, in the same way
        Test.@test S.state_start((state=[0.1, 0.2],), 0.0, 2) == [0.1, 0.2]
        Test.@test S.state_start((state=t -> [t, 2t],), 0.5, 2) == [0.5, 1.0]
        Test.@test S.state_start((control=0.5,), 0.0, 2) == [0.0, 0.0]
        # and it is the value the controls of the problem solved start from
        other = OCP.bryson_denham(backend; init=(state=[0.0, 0.0], control=0.7))
        started = S.prepare(other; grid_size=6).dynamic
        Test.@test S.control_values(started, "u") == fill(0.7, 5)
        Test.@test S.control_values(S.prepare(pb; grid_size=6).dynamic, "u") == zeros(5)

        # 7. prepare: built once, the Optimization problem and the Corleone one
        prepared = S.prepare(pb; grid_size=10)
        Test.@test prepared.problem isa S.Optimization.OptimizationProblem
        Test.@test prepared.dynamic isa S.Corleone.CorleoneDynamicOptProblem

        # 8. solve (smoke: 2 iterations) and outcome
        sol = S.solve(pb, cfg; grid_size=10, max_iter=2)
        Test.@test isfinite(S.objective(sol))
        out = S.outcome(sol)
        Test.@test keys(out) == (:converged, :reason, :objective, :iterations, :violation)
        Test.@test out.objective == S.objective(sol)
        Test.@test isfinite(out.violation) && out.violation >= 0

        # 9. trajectory, from the problem the solve built: the states start at their
        # initial conditions whatever the iterate, and the controls are the solved ones
        traj = S.dynamic_trajectory(sol)
        Test.@test traj isa S.Corleone.Trajectory
        Test.@test length(traj.t) == 10
        Test.@test first(traj.t) == tspan[1] && last(traj.t) == tspan[2]

        tr = S.trajectory(sol, pb)
        Test.@test tr isa Dict{String,Any}
        Test.@test issetequal(
            keys(tr), ("time", "state", "control", "costate", "variable", "names")
        )
        xs = OCP.state_components(sys)
        us = OCP.control_components(sys)
        Test.@test tr["names"]["state"] == xs && tr["names"]["control"] == us
        Test.@test length(tr["time"]) == 10
        Test.@test isempty(tr["costate"]) && isempty(tr["variable"])
        Test.@test all(length(tr["state"][x]) == 10 for x in xs)
        Test.@test all(length(tr["control"][u]) == 10 for u in us)
        # with a free final time, the time axis is the physical one: from t0 to the solved tf
        sol_free = S.solve(free, cfg; grid_size=10, max_iter=2)
        tr_free = S.trajectory(sol_free, free)
        Test.@test tr_free["time"][1] == 0.0
        Test.@test tr_free["time"][end] ≈ tr_free["variable"][1]
        Test.@test tr_free["names"]["variable"] == ["tf"]
        params = OCP.metadata(pb)[:parameters]
        Test.@test tr["state"]["x₁"][1] ≈ params.x1_t0
        Test.@test tr["state"]["x₂"][1] ≈ params.x2_t0
        Test.@test all(isfinite, tr["control"]["u"])
    end
end

end # module TestMethodsModelingToolkit

test_methods() = TestMethodsModelingToolkit.test_methods()
