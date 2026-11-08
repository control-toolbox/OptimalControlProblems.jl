isdefined(Main, :SolveJuMP) || include(joinpath(@__DIR__, "..", "..", "common", "solve_jump.jl"))

module TestSignsJuMP

using Test: Test
using JuMP: JuMP
using OptimalControlProblems: OptimalControlProblems

const OCP = OptimalControlProblems
const EXT = Base.get_extension(OCP, :JuMPModels)
const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# Sign conventions (ct-conventions rules/signs.md, SGN1–SGN6): the multiplier `y` of a
# row `c(x) ∈ S` satisfies `∇F_min + Jᵀy = 0` — `y ≥ 0` when the upper side is active,
# `y ≤ 0` when the lower side is (SGN3), and `min f` and `max -f` have the same
# multipliers (SGN1). `JuMP.dual` is the MOI `ConstraintDual`: `∇F_min − Jᵀy_MOI = 0`,
# a convention fixed by MathOptInterface, independent of the solver and of the
# objective sense, so the accessors return `-JuMP.dual` (`JuMPModels._multiplier`).
# Tested in the two layers of SGN6: the native values on hand-written NLPs, then the
# normalized ones.
const ATOL = 1e-6

# -----------------------------------------------------------------------------
# Hand-written NLPs: `min (x - c)²` with the constraint written in every way the
# rules cover, and `max -(x - c)²` for SGN1. ∇f = 2(x - c) at the solution.
# -----------------------------------------------------------------------------

"""`min` or `max` (as `min -f`) `(x - c)²` with `row` a one-constraint function."""
function nlp(row, c::Real, sense::JuMP.MOI.OptimizationSense)
    model = JuMP.Model()
    JuMP.@variable(model, x)
    ref = row(model, x)
    f = (x - c)^2
    JuMP.@objective(model, sense, sense === JuMP.MOI.MIN_SENSE ? f : -f)
    return model, ref
end

# Equality `x + y = 2`, written straight or negated (the row is the constraint
# function); `∇f = (2, 2)` at the solution `(1, 1)`, `J = (1, 1)` or `(-1, -1)`.
function nlp_equality(; negated::Bool, sense::JuMP.MOI.OptimizationSense)
    model = JuMP.Model()
    JuMP.@variable(model, x)
    JuMP.@variable(model, y)
    ref = negated ? JuMP.@constraint(model, -x - y == -2) : JuMP.@constraint(model, x + y == 2)
    f = x^2 + y^2
    JuMP.@objective(model, sense, sense === JuMP.MOI.MIN_SENSE ? f : -f)
    return model, ref
end

# -----------------------------------------------------------------------------
# An OCP whose costate is known: the integrator `ẋ = u` on `[0, 1]`, `x(0) = 0`,
# `x(1) = 1`, `min ∫u²/2` → `x*(t) = t`, `u* = 1` and `p* ≡ 1` (the pseudo-Hamiltonian
# `H = p u - u²/2` is maximized at `u* = p`, SGN4).
# -----------------------------------------------------------------------------
const INTEGRATOR = EXT.ProblemData(;
    state=[EXT.Component("x")],
    control=[EXT.Component("u")],
    costate=["∂x"],
    times=v -> (0, 1),
    dynamics=(x, u, v, t) -> [u[1]],
    lagrange=(x, u, v, t) -> u[1]^2 / 2,
    boundary=[(0, (x0, xf, v) -> x0[1], 0), (1, (x0, xf, v) -> xf[1], 1)],
)

function test_rows()
    Test.@testset "duals of hand-written rows (Ipopt through MOI, $ATOL)" begin
        # equality: y_MOI = +2 -> y = -2; the negated row gives the opposite (SGN3)
        model, ref = nlp_equality(; negated=false, sense=JuMP.MOI.MIN_SENSE)
        Main.SolveJuMP.solve!(model)
        Test.@test JuMP.dual(ref) ≈ 2 atol = ATOL
        Test.@test EXT._multiplier(ref) ≈ -2 atol = ATOL
        model, ref = nlp_equality(; negated=true, sense=JuMP.MOI.MIN_SENSE)
        Main.SolveJuMP.solve!(model)
        Test.@test EXT._multiplier(ref) ≈ 2 atol = ATOL

        # `min f` and `max -f` have the same multipliers (SGN1): `JuMP.dual` is already
        # the dual of the equivalent minimization, so `_multiplier` is the same too
        model, ref = nlp_equality(; negated=false, sense=JuMP.MOI.MAX_SENSE)
        Main.SolveJuMP.solve!(model)
        Test.@test JuMP.dual(ref) ≈ 2 atol = ATOL
        Test.@test EXT._multiplier(ref) ≈ -2 atol = ATOL

        # upper side `x ≤ 2` active, `min (x - 5)²`: y_MOI = -6 -> y = μU = +6
        model, ref = nlp((m, x) -> JuMP.@constraint(m, x <= 2), 5, JuMP.MOI.MIN_SENSE)
        Main.SolveJuMP.solve!(model)
        Test.@test JuMP.dual(ref) ≈ -6 atol = ATOL
        Test.@test EXT._multiplier(ref) ≈ 6 atol = ATOL

        # lower side `x ≥ -2` active, `min (x + 5)²`: y_MOI = +6 -> y = -μL = -6
        model, ref = nlp((m, x) -> JuMP.@constraint(m, x >= -2), -5, JuMP.MOI.MIN_SENSE)
        Main.SolveJuMP.solve!(model)
        Test.@test EXT._multiplier(ref) ≈ -6 atol = ATOL

        # two-sided row `0 ≤ x ≤ 2`, upper side active: y = μU - μL = +6; the lower
        # side of `-2 ≤ x ≤ 0` gives -6 (SGN3)
        model, ref = nlp((m, x) -> JuMP.@constraint(m, 0 <= x <= 2), 5, JuMP.MOI.MIN_SENSE)
        Main.SolveJuMP.solve!(model)
        Test.@test EXT._multiplier(ref) ≈ 6 atol = ATOL
        model, ref = nlp((m, x) -> JuMP.@constraint(m, -2 <= x <= 0), -5, JuMP.MOI.MIN_SENSE)
        Main.SolveJuMP.solve!(model)
        Test.@test EXT._multiplier(ref) ≈ -6 atol = ATOL

        # `x ≤ 2` as a bound of the variable: the same normalized dual as the row (SGN5)
        model, ref = nlp(
            (m, x) -> (JuMP.set_upper_bound(x, 2); JuMP.UpperBoundRef(x)),
            5,
            JuMP.MOI.MIN_SENSE,
        )
        Main.SolveJuMP.solve!(model)
        Test.@test EXT._multiplier(ref) ≈ 6 atol = ATOL
    end
end

function test_ocp()
    Test.@testset "costate of a model of the collection" begin
        # The integrator, `p* ≡ 1`: the raw `JuMP.dual` is -1, the accessor +1,
        # for every scheme — all of them stamp the multiplier at the step midpoint.
        N = 10
        for scheme in (:trapeze, :euler, :euler_implicit, :midpoint)
            model = EXT.build(INTEGRATOR, N, scheme, :interleaved)
            Main.SolveJuMP.solve!(model)
            p = OCP.costate(model)
            T = collect(Float64, OCP.time_grid(model))
            Test.@test all(isapprox(-1; atol=ATOL), JuMP.dual.(model[:∂x]))
            Test.@test all(t -> isapprox(p(t), 1; atol=ATOL), T[1:(end - 1)])
        end

        # beam, trapezoidal rule: the stationarity `∂H/∂u = 0` of `H = p₁x₂ + p₂u - u²`
        # reads `P_i + P_{i+1} = 4 u_i` on the multiplier `P_i` of step `i` (SGN4), an
        # exact relation of the discrete solution, decisive on the sign. `P_i` is the
        # costate at the midpoint `τ_i` of the step (`_costate_times`).
        N = Main.TestConfigs.SMALL_GRID
        model = OCP.beam(OCP.JuMPBackend(); grid_size=N)
        Main.SolveJuMP.solve!(model)
        p = OCP.costate(model)
        u = OCP.control(model)
        T = collect(Float64, OCP.time_grid(model))
        τ = (T[1:(end - 1)] .+ T[2:end]) ./ 2
        p2(t) = p(t)[2]
        Test.@test [p2(τ[i]) + p2(τ[i + 1]) for i in 1:(N - 1)] ≈
            [4 * u(T[i + 1]) for i in 1:(N - 1)] atol = 1e-3

        # Boundary contract: flat extrapolation outside [τ_1, τ_N]
        Test.@test EXT._costate_times(T) == τ
        Test.@test p(T[1]) == p(τ[1])
        Test.@test p(T[end]) == p(τ[end])
    end
end

function test_signs()
    Test.@testset "Sign of the duals (signs.md, #326)" verbose = VERBOSE showtiming =
        SHOWTIMING begin
        test_rows()
        test_ocp()
    end
end

end # module TestSignsJuMP

test_signs() = TestSignsJuMP.test_signs()
