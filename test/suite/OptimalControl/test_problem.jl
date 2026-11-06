module TestProblemOptimalControl

using Test: Test
using OptimalControl: OptimalControl # loads the OptimalControlModels extension
using OptimalControlProblems: OptimalControlProblems

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
    backend = OCP.OptimalControlBackend()
    Test.@testset "OptimalControl Problem API" verbose = VERBOSE showtiming = SHOWTIMING begin
        Main.TestStatus.for_each_problem("problem", backend) do problem
            pb = getfield(OCP, problem)(backend)
            Test.@test pb isa OCP.Problem
            Test.@test OCP.model(pb) isa OptimalControl.Model
            Test.@test OCP.initial_guess(pb) isa NamedTuple
            Test.@test issubset(keys(OCP.initial_guess(pb)), (:state, :control, :variable))
            data = OCP.metadata(pb)
            Test.@test data[:parameters] == OCP.parameters_data(problem)
            Test.@test data[:status] == OCP.status(problem)
            Test.@test data[:grid_size] == OCP.grid_size_data(problem)

            # The parameters a problem is built with are recorded in its metadata, and the
            # metadata of the collection are left untouched.
            params = OCP.parameters_data(problem)
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
            Test.@test occursin("OptimalControl modeling", shown)
            Test.@test occursin(string(OCP.status(problem)[:OptimalControl]), shown)
            Test.@test occursin(
                "state dimension:    $(OCP.state_dimension(OCP.model(pb)))", shown
            )
            Test.@test sprint(show, pb) == "Problem(:$(problem))"
        end
    end
end

end # module TestProblemOptimalControl

test_problem() = TestProblemOptimalControl.test_problem()
