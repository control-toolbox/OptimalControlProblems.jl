isdefined(Main, :TestCapabilities) ||
    include(joinpath(@__DIR__, "..", "..", "common", "capabilities.jl"))

module TestGPUContract

using Test: Test

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# Source lines of the GPU tests that ask CUDA themselves instead of reading
# `Main.TestCapabilities` (TST6: the predicate is defined once). This file is left out of
# the walk: it spells out the pattern it searches for.
function offenders()
    pattern = Regex("CUDA" * "\\.functional\\(\\)")
    found = Tuple{String,Int}[]
    for file in readdir(@__DIR__)
        (endswith(file, ".jl") && file != basename(@__FILE__)) || continue
        for (n, line) in enumerate(eachline(joinpath(@__DIR__, file)))
            occursin(pattern, line) && push!(found, (file, n))
        end
    end
    return found
end

function test_contract()
    Test.@testset "GPU environment contract" verbose = VERBOSE showtiming = SHOWTIMING begin
        # On a GPU runner a missing device is a failure, not a skip
        if Main.TestCapabilities.ON_GPU_RUNNER
            Test.@test Main.TestCapabilities.CUDA_FUNCTIONAL
        else
            @info "Not on a GPU runner: the GPU tests are skipped when CUDA is not functional" Main.TestCapabilities.CUDA_FUNCTIONAL
        end
        Test.@test isempty(offenders())
    end
end

end # module TestGPUContract

test_contract() = TestGPUContract.test_contract()
