using Aqua
using CTBase
using Ipopt
using JuMP
using NLPModelsIpopt
using OptimalControl
using OptimalControlProblems
using Test

# Parameters
const tol = 1e-6
const mu_strategy = "adaptive"
const sb = "yes"
const constr_viol_tol = 1e-6
const max_iter = 1000
const max_wall_time = 500.0

#
@testset verbose = true showtiming = true "OptimalControlProblems tests" begin
    for name in (
        :aqua, 
        :JuMP, 
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
