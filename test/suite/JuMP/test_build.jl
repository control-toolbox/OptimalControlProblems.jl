# Tests the discretization of the JuMP models, which this package writes: it goes when that code moves to a package of its own (#441).
module TestBuildJuMP

using Test: Test
using JuMP: JuMP
using Ipopt: Ipopt
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

# -----------------------------------------------------------------------------
# The contract of every JuMP model (what the accessors and the tests of the problems rely
# on), checked once here on fake problems: it belongs to `schemes/build.jl`, not to a
# problem. A vector state, control and variable (`DIM1`) next to the scalar ones above.
# -----------------------------------------------------------------------------
const VECTOR = EXT.ProblemData(;
    state=[EXT.Component("x₁"), EXT.Component("x₂")],
    control=[EXT.Component("u₁"), EXT.Component("u₂")],
    variable=[EXT.Component("a"), EXT.Component("T")],
    costate=["∂x₁", "∂x₂"],
    times=v -> (0, v[2]),
    dynamics=(x, u, v, t) -> [x[2] + u[1], u[2] + v[1]],
    lagrange=(x, u, v, t) -> u[1]^2 + u[2]^2,
)

# Number of controls on `N` steps, written here independently of the tableaux: one per node
# for the trapezoidal rule, one per step otherwise.
const CONTROLS = (trapeze=N -> N + 1, euler=N -> N, euler_implicit=N -> N, midpoint=N -> N)

const COMPONENTS = (
    :state_components, :costate_components, :control_components, :variable_components
)

"""
Number of entries of the variable or constraint `name`: the containers of the models are
indexed by the time steps (`0:N`, `0:N-1` or `1:N`), a variable of the problem is a scalar.
"""
function entries(model, name)
    return (c=model[Symbol(name)]; c isa JuMP.AbstractJuMPScalar ? 1 : length(c))
end

function test_contract_generic()
    N = 3
    Test.@testset "the model dictionary" begin
        for (label, data) in (("FAKE", FAKE), ("EXACT", EXACT), ("VECTOR", VECTOR)),
            scheme in SCHEMES,
            ordering in ORDERINGS

            Test.@testset "$label, $scheme, $ordering" begin
                model = EXT.build(data, N, scheme, ordering)
                names = keys(JuMP.object_dictionary(model))
                Test.@test model[:scheme] === scheme
                Test.@test model[:ordering] === ordering
                Test.@test model[:problem_data] === data
                Test.@test :time_grid ∈ names
                Test.@test model[:time_grid] isa Function
                Test.@test length(model[:constraints]) == JuMP.num_constraints(
                    model; count_variable_in_set_constraints=false
                )
                for key in COMPONENTS
                    Test.@test model[key] isa Vector{String}
                    for name in model[key]
                        Test.@test Symbol(name) ∈ names
                    end
                end
                Test.@test length(model[:costate_components]) ==
                    length(model[:state_components])
                # state at the N+1 nodes, controls per scheme, one costate (dual of a
                # dynamics constraint) per step, a scalar per variable
                for name in model[:state_components]
                    Test.@test entries(model, name) == N + 1
                end
                for name in model[:control_components]
                    Test.@test entries(model, name) == CONTROLS[scheme](N)
                end
                for name in model[:costate_components]
                    Test.@test entries(model, name) == N
                end
                for name in model[:variable_components]
                    Test.@test entries(model, name) == 1
                end
            end
        end
    end

    Test.@testset "the time grid" begin
        for scheme in SCHEMES, ordering in ORDERINGS
            # fixed times: no solve, nothing to give
            model = EXT.build(EXACT, N, scheme, ordering)
            Test.@test collect(model[:time_grid]()) ≈ [0, 1 / 3, 2 / 3, 1]
            # free final time: unknown until a solve, or read at the start values
            model = EXT.build(FAKE, N, scheme, ordering; init=(variable=1.5,))
            Test.@test_throws JuMP.OptimizeNotCalled model[:time_grid]()
            Test.@test collect(model[:time_grid](JuMP.start_value)) ≈ [0, 0.5, 1.0, 1.5]
            model = EXT.build(VECTOR, N, scheme, ordering; init=(variable=[0.5, 3.0],))
            Test.@test collect(model[:time_grid](JuMP.start_value)) ≈ [0, 1, 2, 3]
        end
    end

    Test.@testset "unknown scheme or ordering" begin
        Test.@test_throws CTBase.Exceptions.IncorrectArgument EXT.build(
            LINEAR, N, :gauss_legendre_2, :interleaved
        )
        Test.@test_throws CTBase.Exceptions.IncorrectArgument EXT.build(
            LINEAR, N, :trapeze, :random
        )
    end
end

# -----------------------------------------------------------------------------
# The start values: the initial guess (#377) is loaded as CTDirect does, the state at the
# N+1 nodes, the control at the nodes of its M values, the variable once.
# -----------------------------------------------------------------------------
"""Start value of every variable of `model`, by name."""
function starts(model)
    return Dict(JuMP.name(x) => JuMP.start_value(x) for x in JuMP.all_variables(model))
end

"""Start values of the container `name`, in the order of the nodes."""
start_of(model, name) = JuMP.start_value.(collect(model[Symbol(name)]))

function test_start()
    N = 2
    Test.@testset "start values" begin
        for scheme in SCHEMES
            M = CONTROLS[scheme](N)
            Test.@testset "$scheme" begin
                # no guess: nothing is set
                model = EXT.build(VECTOR, N, scheme, :interleaved)
                Test.@test all(isnothing, values(starts(model)))

                # constants, vectors for several components
                init = (state=[1.0, 2.0], control=[3.0, 4.0], variable=[5.0, 6.0])
                model = EXT.build(VECTOR, N, scheme, :interleaved; init=init)
                Test.@test start_of(model, "x₁") == fill(1.0, N + 1)
                Test.@test start_of(model, "x₂") == fill(2.0, N + 1)
                Test.@test start_of(model, "u₁") == fill(3.0, M)
                Test.@test start_of(model, "u₂") == fill(4.0, M)
                Test.@test JuMP.start_value(model[:a]) == 5.0
                Test.@test JuMP.start_value(model[:T]) == 6.0

                # a scalar is the guess of a one-dimensional component
                model = EXT.build(
                    FAKE,
                    N,
                    scheme,
                    :interleaved;
                    init=(state=0.5, control=-0.5, variable=1.6),
                )
                Test.@test start_of(model, "x") == fill(0.5, N + 1)
                Test.@test start_of(model, "u") == fill(-0.5, M)
                Test.@test JuMP.start_value(model[:T]) == 1.6

                # a key that is missing leaves its variables unset
                model = EXT.build(VECTOR, N, scheme, :interleaved; init=(state=[1.0, 2.0],))
                Test.@test all(isnothing, start_of(model, "u₁"))
                Test.@test isnothing(JuMP.start_value(model[:T]))
                Test.@test start_of(model, "x₁") == fill(1.0, N + 1)

                # a function of time, on the grid of the guess of the final time (2.0): the
                # nodes are 0, 1, 2
                init = (
                    state=t -> [t, 2t], control=t -> [t + 1, t + 2], variable=[5.0, 2.0]
                )
                model = EXT.build(VECTOR, N, scheme, :interleaved; init=init)
                Test.@test start_of(model, "x₁") == [0.0, 1.0, 2.0]
                Test.@test start_of(model, "x₂") == [0.0, 2.0, 4.0]
                Test.@test start_of(model, "u₁") == (0:(M - 1)) .+ 1.0
                Test.@test start_of(model, "u₂") == (0:(M - 1)) .+ 2.0

                # the ordering changes the order of the variables, not their start values
                blocks = EXT.build(VECTOR, N, scheme, :blocks; init=init)
                Test.@test starts(blocks) == starts(model)
            end
        end
    end
end

# -----------------------------------------------------------------------------
# The arguments of `JuMP.Model` are forwarded by `build`.
# -----------------------------------------------------------------------------
function test_forwarding()
    Test.@testset "arguments of JuMP.Model" begin
        model = EXT.build(FAKE, 2, :trapeze, :interleaved, Ipopt.Optimizer)
        Test.@test JuMP.solver_name(model) == "Ipopt"
        Test.@test JuMP.bridge_constraints(model)
        model = EXT.build(
            FAKE, 2, :trapeze, :interleaved, Ipopt.Optimizer; add_bridges=false
        )
        Test.@test JuMP.solver_name(model) == "Ipopt"
        Test.@test !JuMP.bridge_constraints(model)
    end
end

function test_build()
    Test.@testset "JuMP schemes: the discretization" verbose = VERBOSE showtiming =
        SHOWTIMING begin
        test_by_hand()
        test_order()
        test_orderings()
        test_contract_generic()
        test_start()
        test_forwarding()
    end
end

end # module TestBuildJuMP

test_build() = TestBuildJuMP.test_build()
