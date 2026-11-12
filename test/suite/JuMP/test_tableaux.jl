# Tests the discretization of the JuMP models, which this package writes: it goes when that code moves to a package of its own (#441).
module TestTableauxJuMP

using Test: Test
using JuMP: JuMP # loads the JuMPModels extension
using CTBase: CTBase
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# The properties of the four tableaux (#369), written here by hand from their definition:
# the extension computes them from (A, b, c).
const EXPECTED = (
    trapeze=(fsal=true, stiffly_accurate=true, explicit=false, controls=11, θ=[0.0, 1.0]),
    euler=(fsal=false, stiffly_accurate=false, explicit=true, controls=10, θ=[0.0]),
    euler_implicit=(
        fsal=false, stiffly_accurate=true, explicit=false, controls=10, θ=[1.0]
    ),
    midpoint=(fsal=false, stiffly_accurate=false, explicit=false, controls=10, θ=[0.5]),
)

# Control used by the path constraints at the nodes 0, 1, N-1, N of a grid of N = 10 steps.
const AT_NODES = (
    trapeze=[0, 1, 9, 10],
    euler=[0, 1, 9, 9],
    euler_implicit=[0, 0, 8, 9],
    midpoint=[0, 1, 9, 9],
)

function test_tableaux()
    ext = Base.get_extension(OptimalControlProblems, :JuMPModels)
    N = 10
    Test.@testset "JuMP schemes as Butcher tableaux" verbose = VERBOSE showtiming =
        SHOWTIMING begin
        for (scheme, e) in pairs(EXPECTED)
            Test.@testset "$scheme" begin
                tab = ext.tableau(scheme)
                Test.@test ext.is_fsal(tab) == e.fsal
                Test.@test ext.is_stiffly_accurate(tab) == e.stiffly_accurate
                Test.@test ext.is_explicit(tab) == e.explicit
                Test.@test ext.has_final_control(tab) == e.fsal
                Test.@test ext.control_count(tab, N) == e.controls
                Test.@test ext.stage_positions(tab) == e.θ
                Test.@test ext.stage_reuse(tab) isa
                    (e.fsal ? ext.FirstSameAsLast : ext.NoStageReuse)
                Test.@test [ext.control_at_node(tab, k, N) for k in (0, 1, N - 1, N)] == AT_NODES[scheme]
            end
        end

        Test.@testset "a stage inside the step needs stage unknowns" begin
            # Hermite–Simpson (Lobatto IIIA, 3 stages): the middle stage is not a node
            A = [0 0 0; 5 / 24 1 / 3 -1 / 24; 1 / 6 2 / 3 1 / 6]
            tab = ext.ButcherTableau(:lobatto_iiia_3, A, [1 / 6, 2 / 3, 1 / 6], [0, 0.5, 1])
            Test.@test ext.is_fsal(tab)
            Test.@test_throws CTBase.Exceptions.NotImplemented ext.stage_positions(tab)
        end

        Test.@testset "unknown scheme" begin
            Test.@test_throws CTBase.Exceptions.IncorrectArgument ext.tableau(:rk4)
        end
    end
end

end # module TestTableauxJuMP

test_tableaux() = TestTableauxJuMP.test_tableaux()
