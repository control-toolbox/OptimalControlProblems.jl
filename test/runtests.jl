using Test
using OptimalControlProblems

#
@testset verbose = true showtiming = true "OptimalControlProblems tests" begin
    for name ∈ (
        :JuMP_optimality,
        :OptimalControl_optimality,
        )
        @testset "$(name)" begin
            test_name = Symbol(:test_, name)
            include("$(test_name).jl")
            @eval $test_name()
        end
    end
end
