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

        # 1. _extract_tspan
        tspan = Main.SolveModelingToolkit._extract_tspan(sys)
        Test.@test tspan == (0.0, 1.0)

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
        params = OCP.metadata(pb)[:parameters]
        Test.@test tr["state"]["x₁"][1] ≈ params.x1_t0
        Test.@test tr["state"]["x₂"][1] ≈ params.x2_t0
        Test.@test all(isfinite, tr["control"]["u"])
    end
end

end # module TestMethodsModelingToolkit

test_methods() = TestMethodsModelingToolkit.test_methods()
