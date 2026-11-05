isdefined(Main, :TestMethods) || include(joinpath(@__DIR__, "..", "..", "common", "methods.jl"))

module TestMethodsOptimalControl

using Test: Test
using OptimalControl: OptimalControl # loads the OptimalControlModels extension
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

function test_methods()
    Test.@testset "OptimalControl methods of the extension" verbose = VERBOSE showtiming = SHOWTIMING begin
        Main.TestMethods.test_methods(OptimalControlProblems.OptimalControlBackend(), :OptimalControlModels)
    end
end

end # module TestMethodsOptimalControl

test_methods() = TestMethodsOptimalControl.test_methods()
