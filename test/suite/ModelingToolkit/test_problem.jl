module TestProblemModelingToolkit

using Test: Test
using ModelingToolkit: ModelingToolkit as MTK
using OptimalControlProblems: OptimalControlProblems
using CTModels: CTModels
using CTBase: CTBase

const OCP = OptimalControlProblems
const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

"""
`params` with its last nonzero numerical value changed by a relative 1e-6, small enough to
keep every bound of the problem ordered; `nothing` if there is no such value.
"""
function changed(params::NamedTuple)
    k = findlast(v -> v isa AbstractFloat || (v isa Integer && v != 0), values(params))
    isnothing(k) && return nothing
    key = keys(params)[k]
    return merge(params, NamedTuple{(key,)}((params[key] * (1 + 1e-6),)))
end

function test_problem()
    backend = OCP.ModelingToolkitBackend()
    Test.@testset "ModelingToolkit Problem API" verbose = VERBOSE showtiming = SHOWTIMING begin
        Main.TestStatus.for_each_problem("problem", backend) do problem
            pb = getfield(OCP, problem)(backend)
            Test.@test pb isa OCP.Problem
            Test.@test OCP.model(pb) isa MTK.AbstractSystem
            Test.@test OCP.initial_guess(pb) isa NamedTuple
            Test.@test issubset(keys(OCP.initial_guess(pb)), (:state, :control, :variable))
            data = OCP.metadata(pb)
            Test.@test data[:parameters] == OCP.parameters_data(problem)
            Test.@test data[:status] == OCP.status(problem)
            Test.@test data[:grid_size] == OCP.grid_size_data(problem)

            # Check CTModels dimension and component methods
            sys = OCP.model(pb)
            Test.@test CTModels.Models.state_dimension(sys) == length(CTModels.Models.state_components(sys))
            Test.@test CTModels.Models.control_dimension(sys) == length(CTModels.Models.control_components(sys))
            Test.@test CTModels.Models.variable_dimension(sys) == length(CTModels.Models.variable_components(sys))
            Test.@test CTModels.Models.state_dimension(sys) > 0
            Test.@test CTModels.Models.control_dimension(sys) > 0

            # Verify MTK model equations and structure
            eqs = MTK.equations(sys)
            Test.@test length(eqs) == CTModels.Models.state_dimension(sys)
            iv = MTK.independent_variable(sys)
            Test.@test iv !== nothing

            # Verify costs
            costs = MTK.get_costs(sys)
            Test.@test !isempty(costs)
            cost = costs[1]
            op = MTK.SymbolicUtils.operation(cost)
            Test.@test hasproperty(op, :domain)
            d = op.domain.domain
            params = OCP.parameters_data(problem)
            Test.@test Float64(d.left) == Float64(params.t0)
            Test.@test Float64(d.right) == Float64(params.tf)

            # Verify constraints
            cons = MTK.get_constraints(sys)
            Test.@test !isempty(cons)

            # The parameters a problem is built with are recorded in its metadata, and the
            # metadata of the collection are left untouched.
            new = isnothing(params) ? nothing : changed(params)
            if !isnothing(new)
                Test.@test new != params
                Test.@test OCP.metadata(getfield(OCP, problem)(backend; parameters=new))[:parameters] ==
                    new
                Test.@test OCP.parameters_data(problem) == params
            end

            # The initial guess is the one of the metadata (#377), unless another one is given.
            Test.@test OCP.initial_guess(pb) == OCP.initial_guess_data(problem)
            other = (state=OCP.initial_guess_data(problem).state, control=0)
            Test.@test OCP.initial_guess(getfield(OCP, problem)(backend; init=other)) === other

            # Shown with its name, modeling, status and dimensions.
            shown = sprint(show, MIME("text/plain"), pb)
            Test.@test occursin(string(problem), shown)
            Test.@test occursin("ModelingToolkit modeling", shown)
            Test.@test occursin(string(OCP.status(problem)[:ModelingToolkit]), shown)
            Test.@test occursin(
                "state dimension:    $(CTModels.Models.state_dimension(sys))", shown
            )
            Test.@test sprint(show, pb) == "Problem(:$(problem))"
        end
    end
end

end # module TestProblemModelingToolkit

test_problem() = TestProblemModelingToolkit.test_problem()
