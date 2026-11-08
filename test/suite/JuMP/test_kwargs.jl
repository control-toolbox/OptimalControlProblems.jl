module TestKwargsJuMP

using Test: Test
using JuMP: JuMP
using Ipopt: Ipopt
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

function test_kwargs()
    Test.@testset "JuMP keyword arguments" verbose = VERBOSE showtiming = SHOWTIMING begin
        Main.TestStatus.for_each_problem(
            "kwargs", OptimalControlProblems.JuMPBackend()
        ) do problem
            build = getfield(OptimalControlProblems, problem)
            # arguments of the model constructor are forwarded to JuMP.Model
            model = build(
                OptimalControlProblems.JuMPBackend(), Ipopt.Optimizer; add_bridges=true
            )
            Test.@test JuMP.solver_name(model) == "Ipopt"
            # the initial guess of the metadata is the start, unless another one is given (#377)
            default = build(OptimalControlProblems.JuMPBackend(); grid_size=4)
            Test.@test all(!isnothing, JuMP.start_value.(JuMP.all_variables(default)))
            # state and control doubled; the variable kept, so that the time grid is the same
            guess = OptimalControlProblems.initial_guess_data(problem)
            double(g) = g isa Function ? (t -> 2 .* g(t)) : 2 .* g
            other = merge(guess, (state=double(guess.state), control=double(guess.control)))
            changed = build(OptimalControlProblems.JuMPBackend(); grid_size=4, init=other)
            fixed = Set(OptimalControlProblems.variable_components(default))
            factor = [JuMP.name(x) in fixed ? 1 : 2 for x in JuMP.all_variables(default)]
            Test.@test JuMP.start_value.(JuMP.all_variables(changed)) ≈
                factor .* JuMP.start_value.(JuMP.all_variables(default))
        end
    end
end

end # module TestKwargsJuMP

test_kwargs() = TestKwargsJuMP.test_kwargs()
