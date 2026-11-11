module TestContractJuMP

using Test: Test
using JuMP: JuMP
using Ipopt: Ipopt
using OptimalControlProblems: OptimalControlProblems

const OCP = OptimalControlProblems
const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# What a problem file writes, checked on small models and without a solve: its components, its
# default initial guess and its time grid; and that it forwards every argument to `build`
# (`scheme`, `ordering`, `init`, the arguments of `JuMP.Model`). The transcription itself, the
# same for every problem, is tested once in `test_build.jl` (#441).

"""Start value of every variable of `model`, by name."""
function starts(model)
    return Dict(JuMP.name(x) => JuMP.start_value(x) for x in JuMP.all_variables(model))
end

"""Number of nodes of the state of `model`."""
nodes(model) = length(model[Symbol(first(model[:state_components]))])

function test_contract()
    Test.@testset "JuMP problems (no solve)" verbose = VERBOSE showtiming = SHOWTIMING begin
        Main.TestStatus.for_each_problem("contract", OCP.JuMPBackend()) do problem
            build = getfield(OCP, problem)

            # the default model: components, initial guess of the metadata (#377), time grid
            model = build(OCP.JuMPBackend(); grid_size=2)
            names = [model[:state_components]; model[:control_components]; model[:variable_components]]
            Test.@test !isempty(model[:state_components])
            Test.@test !isempty(model[:control_components])
            Test.@test allunique(names)
            Test.@test length(model[:costate_components]) == length(model[:state_components])
            Test.@test all(x -> JuMP.start_value(x) isa Real && isfinite(JuMP.start_value(x)),
                JuMP.all_variables(model))
            # the initial time grid, read at the start values: no solve, even for a free time
            T = collect(model[:time_grid](JuMP.start_value))
            Test.@test length(T) == 3
            Test.@test issorted(T) && T[1] < T[end]

            # every argument is forwarded: scheme, ordering, grid, initial guess (state and
            # control doubled, the variable kept so that the time grid is the same) and the
            # arguments of `JuMP.Model`
            guess = OCP.initial_guess_data(problem)
            double(g) = g isa Function ? (t -> 2 .* g(t)) : 2 .* g
            other = merge(guess, (state=double(guess.state), control=double(guess.control)))
            options = (grid_size=3, scheme=:midpoint, ordering=:blocks)
            base = build(OCP.JuMPBackend(); options...)
            changed = build(OCP.JuMPBackend(), Ipopt.Optimizer; options..., init=other, add_bridges=false)
            Test.@test changed[:scheme] === :midpoint
            Test.@test changed[:ordering] === :blocks
            Test.@test nodes(changed) == 4
            Test.@test JuMP.solver_name(changed) == "Ipopt"
            Test.@test !JuMP.bridge_constraints(changed)
            fixed = Set(model[:variable_components])
            reference, doubled = starts(base), starts(changed)
            Test.@test keys(doubled) == keys(reference)
            Test.@test all(
                doubled[name] ≈ (name in fixed ? 1 : 2) * reference[name] for name in keys(reference)
            )
        end
    end
end

end # module TestContractJuMP

test_contract() = TestContractJuMP.test_contract()
