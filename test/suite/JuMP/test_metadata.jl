module TestMetadataJuMP

using Test: Test
using JuMP: JuMP
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# Small grid: only the structure of the model is checked, nothing is solved.
const GRID_SIZE = 10

const REQUIRED = (
    :time_grid,
    :state_components,
    :costate_components,
    :control_components,
    :variable_components,
)

function test_metadata()
    Test.@testset "JuMP model information" verbose = VERBOSE showtiming = SHOWTIMING begin
        for problem in OptimalControlProblems.problems(OptimalControlProblems.JuMPBackend())
            Test.@testset "$problem" begin
                build = getfield(OptimalControlProblems, problem)
                model = build(OptimalControlProblems.JuMPBackend(); grid_size=GRID_SIZE)
                keys_ = keys(JuMP.object_dictionary(model))

                # required information is present
                for key in REQUIRED
                    Test.@test key ∈ keys_
                end

                # every component name is the name of a variable or of a constraint
                for key in REQUIRED[2:end]
                    names = model[key]
                    isnothing(names) && continue
                    for name in names
                        Test.@test Symbol(name) ∈ keys_
                    end
                end
            end
        end
    end
end

end # module TestMetadataJuMP

test_metadata() = TestMetadataJuMP.test_metadata()
