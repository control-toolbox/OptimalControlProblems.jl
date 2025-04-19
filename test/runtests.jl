using Aqua
using CTBase
using Ipopt
using JuMP
using NLPModelsIpopt
using OptimalControl
using OptimalControlProblems
using Test

#
@testset verbose = true showtiming = true "OptimalControlProblems tests" begin
    for name in (
        #:aqua, 
        #:JuMP, 
        :OptimalControl,
        )
        @testset "$(name)" begin
            test_name = Symbol(:test_, name)
            println("Testing: " * string(name))
            include("$(test_name).jl")
            @eval $test_name()
        end
    end
end
