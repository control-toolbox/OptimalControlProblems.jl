using Aqua
using CTBase
using CTDirect
using Ipopt
using JuMP
using NLPModelsIpopt
using OptimalControlProblems
using Test

#
@testset verbose = true showtiming = true "OptimalControlProblems tests" begin
    for name in (:aqua, :JuMP_optimality, :OptimalControl_optimality)
        @testset "$(name)" begin
            test_name = Symbol(:test_, name)
            include("$(test_name).jl")
            @eval $test_name()
        end
    end
end
