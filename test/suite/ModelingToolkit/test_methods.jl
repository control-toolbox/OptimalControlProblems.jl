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

        # 5. settings
        cfg = (shooting=:single, solver=:ipopt)
        stg = Main.SolveModelingToolkit.settings(cfg; max_iter=5)
        Test.@test stg.solver === :ipopt
        Test.@test stg.shooting === :single
        Test.@test stg.options.max_iter == 5

        # 6. prepare
        opt_prob = Main.SolveModelingToolkit.prepare(pb; grid_size=10)
        Test.@test opt_prob isa Main.SolveModelingToolkit.Optimization.OptimizationProblem

        # 7. solve (smoke: 2 iterations)
        res = Main.SolveModelingToolkit.solve(pb, cfg; grid_size=10, max_iter=2)
        Test.@test isfinite(res.objective)
        Test.@test hasproperty(res, :retcode)
        out = Main.SolveModelingToolkit.outcome(res)
        Test.@test haskey(out, :converged)
        Test.@test haskey(out, :objective)
        Test.@test isfinite(out.objective)
    end
end

end # module TestMethodsModelingToolkit

test_methods() = TestMethodsModelingToolkit.test_methods()
