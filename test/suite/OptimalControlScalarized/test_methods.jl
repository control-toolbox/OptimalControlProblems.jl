isdefined(Main, :TestMethods) ||
    include(joinpath(@__DIR__, "..", "..", "common", "methods.jl"))

module TestMethodsOptimalControlScalarized

using Test: Test
using OptimalControl: OptimalControl # loads the OptimalControlModels extension
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

function test_methods()
    Test.@testset "OptimalControlScalarized methods of the extension" verbose = VERBOSE showtiming =
        SHOWTIMING begin
        Main.TestMethods.test_methods(
            OptimalControlProblems.OptimalControlScalarizedBackend(), :OptimalControlModels
        )
    end
end

end # module TestMethodsOptimalControlScalarized

test_methods() = TestMethodsOptimalControlScalarized.test_methods()
