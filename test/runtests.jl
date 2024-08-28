using Test
using CTApp

#
@testset verbose = true showtiming = true "Base" begin
    for name ∈ (
        :default,
        )
        @testset "$(name)" begin
            test_name = Symbol(:test_, name)
            include("$(test_name).jl")
            @eval $test_name()
        end
    end
end
