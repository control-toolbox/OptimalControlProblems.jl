module TestAqua

using Test: Test
using Aqua: Aqua
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

function test_aqua()
    Test.@testset "Aqua.jl" verbose = VERBOSE showtiming = SHOWTIMING begin
        Aqua.test_all(
            OptimalControlProblems;
            ambiguities=false,
            deps_compat=(ignore=[:LinearAlgebra, :Unicode],),
            piracies=true,
        )
        # do not warn about ambiguities in dependencies
        Aqua.test_ambiguities(OptimalControlProblems)
    end
end

end # module TestAqua

test_aqua() = TestAqua.test_aqua()
