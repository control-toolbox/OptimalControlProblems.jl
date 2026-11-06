module TestBuildJuMP

using Test: Test
using JuMP: JuMP
using CTBase: CTBase
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

const EXT = Base.get_extension(OptimalControlProblems, :JuMPModels)
const SCHEMES = (:trapeze, :euler, :euler_implicit, :midpoint)
const ORDERINGS = (:interleaved, :blocks)

# -----------------------------------------------------------------------------
# A fake problem, small enough to write its NLP by hand: free final time `T`,
#   ẋ = x u + t,   ∫ (u² + t x) dt,   -10 ≤ x + u ≤ 10,   x(0) = 1.
# -----------------------------------------------------------------------------
f(x, u, t) = x * u + t
L(x, u, t) = u^2 + t * x
c(x, u) = x + u

const FAKE = EXT.ProblemData(;
    state=[EXT.Component("x")],
    control=[EXT.Component("u")],
    variable=[EXT.Component("T")],
    costate=["∂x"],
    times=v -> (0, v[1]),
    dynamics=(x, u, v, t) -> [f(x[1], u[1], t)],
    lagrange=(x, u, v, t) -> L(x[1], u[1], t),
    path=[(-10, (x, u, v, t) -> c(x[1], u[1]), 10)],
    boundary=[(1, (x0, xf, v) -> x0[1], 1)],
)

# The point where the NLP is evaluated, N = 2 steps.
const X = [1.0, 1.3, 0.7]
const U = [0.4, -0.2, 0.9]       # the last one exists only for the trapezoidal rule
const T = 1.6

"""The NLP of the fake problem written by hand: dynamics residuals, path values, cost."""
function by_hand(scheme)
    h = T / 2
    t = [0, h, T]
    if scheme == :trapeze
        dyn = [
            X[i + 1] - X[i] -
            h / 2 * (f(X[i], U[i], t[i]) + f(X[i + 1], U[i + 1], t[i + 1])) for i in 1:2
        ]
        path = [c(X[k], U[k]) for k in 1:3]
        cost = sum(
            h / 2 * (L(X[i], U[i], t[i]) + L(X[i + 1], U[i + 1], t[i + 1])) for i in 1:2
        )
    elseif scheme == :euler
        dyn = [X[i + 1] - X[i] - h * f(X[i], U[i], t[i]) for i in 1:2]
        path = [c(X[1], U[1]), c(X[2], U[2]), c(X[3], U[2])]
        cost = sum(h * L(X[i], U[i], t[i]) for i in 1:2)
    elseif scheme == :euler_implicit
        dyn = [X[i + 1] - X[i] - h * f(X[i + 1], U[i], t[i + 1]) for i in 1:2]
        path = [c(X[1], U[1]), c(X[2], U[1]), c(X[3], U[2])]
        cost = sum(h * L(X[i + 1], U[i], t[i + 1]) for i in 1:2)
    else # midpoint
        m(i) = (X[i] + X[i + 1]) / 2
        dyn = [X[i + 1] - X[i] - h * f(m(i), U[i], t[i] + h / 2) for i in 1:2]
        path = [c(X[1], U[1]), c(X[2], U[2]), c(X[3], U[2])]
        cost = sum(h * L(m(i), U[i], t[i] + h / 2) for i in 1:2)
    end
    return (nvar=(scheme == :trapeze ? 3 : 2) + 3 + 1, dyn=dyn, path=sort(path), cost=cost)
end

"""Value of `ex` with the variables of `model` at the point above."""
function evaluate(model, ex)
    values = Dict{JuMP.VariableRef,Float64}()
    for (k, x) in enumerate(collect(model[:x]))
        values[x] = X[k]
    end
    for (k, u) in enumerate(collect(model[:u]))
        values[u] = U[k]
    end
    values[model[:T]] = T
    return JuMP.value(v -> values[v], ex)
end

"""Value of a constraint, `func - set` for an equality, `func` for an interval."""
function constraint_value(model, ref)
    o = JuMP.constraint_object(ref)
    v = evaluate(model, o.func)
    return o.set isa JuMP.MOI.EqualTo ? v - o.set.value : v
end

function test_by_hand()
    Test.@testset "the NLP of a fake problem, written by hand" begin
        for scheme in SCHEMES, ordering in ORDERINGS
            Test.@testset "$scheme, $ordering" begin
                model = EXT.build(FAKE, 2, scheme, ordering)
                e = by_hand(scheme)
                Test.@test JuMP.num_variables(model) == e.nvar
                dyn = [constraint_value(model, r) for r in collect(model[:∂x])]
                Test.@test dyn ≈ e.dyn atol = 1e-14
                path = [
                    constraint_value(model, r) for r in JuMP.all_constraints(
                        model,
                        JuMP.GenericNonlinearExpr{JuMP.VariableRef},
                        JuMP.MOI.Interval{Float64},
                    )
                ]
                append!(
                    path,
                    [
                        constraint_value(model, r) for r in JuMP.all_constraints(
                            model, JuMP.AffExpr, JuMP.MOI.Interval{Float64}
                        )
                    ],
                )
                Test.@test sort(path) ≈ e.path atol = 1e-14
                Test.@test evaluate(model, JuMP.objective_function(model)) ≈ e.cost atol =
                    1e-14
            end
        end
    end
end

# -----------------------------------------------------------------------------
# Order of the schemes: the exact solution x(t) = exp(t²/2) of ẋ = t x is put at the nodes
# (no solve). The dynamics residual is the local error, O(h^{p+1}); the cost is a quadrature
# of ∫₀¹ t x dt = e^{1/2} - 1, with an error O(h^p); p is the order of the scheme.
# -----------------------------------------------------------------------------
const ORDER = (trapeze=2, euler=1, euler_implicit=1, midpoint=2)

const EXACT = EXT.ProblemData(;
    state=[EXT.Component("x")],
    control=[EXT.Component("u")],
    costate=["∂x"],
    times=v -> (0, 1),
    dynamics=(x, u, v, t) -> [t * x[1]],
    lagrange=(x, u, v, t) -> t * x[1],
)

function errors(scheme, N)
    model = EXT.build(EXACT, N, scheme, :interleaved)
    values = Dict{JuMP.VariableRef,Float64}()
    for (k, x) in enumerate(collect(model[:x]))
        values[x] = exp(((k - 1) / N)^2 / 2)
    end
    for u in collect(model[:u])
        values[u] = 0.0
    end
    residual = maximum(
        abs(JuMP.value(v -> values[v], JuMP.constraint_object(r).func)) for
        r in collect(model[:∂x])
    )
    cost = abs(JuMP.value(v -> values[v], JuMP.objective_function(model)) - (exp(0.5) - 1))
    return residual, cost
end

function test_order()
    Test.@testset "order of the schemes" begin
        for (scheme, p) in pairs(ORDER)
            Test.@testset "$scheme" begin
                (r1, c1), (r2, c2) = errors(scheme, 40), errors(scheme, 80)
                Test.@test log2(r1 / r2) ≈ p + 1 atol = 0.1
                Test.@test log2(c1 / c2) ≈ p atol = 0.1
            end
        end
    end
end

# -----------------------------------------------------------------------------
# Orderings (#370), written by hand for N = 2. MOI groups the constraints by type, so the
# order of creation shows only between constraints of the same type: in this linear fake
# problem the dynamics, the second path constraint and the boundary condition are all
# affine equalities.
# -----------------------------------------------------------------------------
const LINEAR = EXT.ProblemData(;
    state=[EXT.Component("x")],
    control=[EXT.Component("u")],
    variable=[EXT.Component("T")],
    costate=["∂x"],
    times=v -> (0, 1),
    dynamics=(x, u, v, t) -> [u[1]],
    path=[(-10, (x, u, v, t) -> x[1] + u[1], 10), (0, (x, u, v, t) -> x[1] - u[1], 0)],
    boundary=[(1, (x0, xf, v) -> x0[1], 1)],
)

function variable_names(scheme, ordering)
    x = ["x[0]", "x[1]", "x[2]"]
    u = scheme == :trapeze ? ["u[0]", "u[1]", "u[2]"] : ["u[0]", "u[1]"]
    ordering == :blocks && return [x; u; "T"]
    interleaved = if scheme == :trapeze
        ["x[0]", "u[0]", "x[1]", "u[1]", "x[2]", "u[2]"]
    else
        ["x[0]", "u[0]", "x[1]", "u[1]", "x[2]"]
    end
    return [interleaved; "T"]
end

# `model[:constraints]` lists, in CTDirect's order: the dynamics of step 1 (1), the path
# constraints at node 0 (2, 3), the dynamics of step 2 (4), the path constraints at nodes 1
# and 2 (5 to 8), the boundary condition (9). The positions in the order of creation:
const CREATED = (interleaved=1:9, blocks=[1, 4, 2, 3, 5, 6, 7, 8, 9])

"""Value of the function of the constraint `ref` at a point given by the variable names."""
function by_name(ref)
    point(v) = 0.1 * findfirst(==(JuMP.name(v)), variable_names(:trapeze, :blocks))
    return JuMP.value(point, JuMP.constraint_object(ref).func)
end

function test_orderings()
    Test.@testset "orderings" begin
        for scheme in SCHEMES
            reference = EXT.build(LINEAR, 2, scheme, :interleaved)
            for ordering in ORDERINGS
                Test.@testset "$scheme, $ordering" begin
                    model = EXT.build(LINEAR, 2, scheme, ordering)
                    Test.@test model[:ordering] === ordering
                    Test.@test JuMP.name.(JuMP.all_variables(model)) ==
                        variable_names(scheme, ordering)
                    C = model[:constraints]
                    Test.@test length(C) == 9
                    Test.@test length(C) == JuMP.num_constraints(
                        model; count_variable_in_set_constraints=false
                    )
                    created = C[CREATED[ordering]]
                    for (F, S) in JuMP.list_of_constraint_types(model)
                        F <: JuMP.VariableRef && continue
                        Test.@test JuMP.all_constraints(model, F, S) == filter(
                            r -> r in JuMP.all_constraints(model, F, S), created
                        )
                    end
                    # the same constraints as the interleaved model, in CTDirect's order
                    Test.@test [JuMP.constraint_object(r).set for r in C] == [
                        JuMP.constraint_object(r).set for r in reference[:constraints]
                    ]
                    Test.@test by_name.(C) ≈ by_name.(reference[:constraints]) atol = 1e-14
                end
            end
        end
        Test.@test_throws CTBase.Exceptions.IncorrectArgument EXT.build(
            LINEAR, 2, :trapeze, :random
        )
    end
end

function test_build()
    Test.@testset "JuMP schemes: the discretization" verbose = VERBOSE showtiming =
        SHOWTIMING begin
        test_by_hand()
        test_order()
        test_orderings()
    end
end

end # module TestBuildJuMP

test_build() = TestBuildJuMP.test_build()
