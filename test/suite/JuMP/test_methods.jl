isdefined(Main, :TestMethods) ||
    include(joinpath(@__DIR__, "..", "..", "common", "methods.jl"))

module TestMethodsJuMP

using Test: Test
using JuMP: JuMP # loads the JuMPModels extension
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

function test_methods()
    Test.@testset "JuMP methods of the extension" verbose = VERBOSE showtiming = SHOWTIMING begin
        Main.TestMethods.test_methods(OptimalControlProblems.JuMPBackend(), :JuMPModels)
    end
end

end # module TestMethodsJuMP

test_methods() = TestMethodsJuMP.test_methods()
