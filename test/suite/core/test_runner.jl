module TestRunner

using Test: Test

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# The runner is a plain file shared by the dispatcher and every environment.
include(joinpath(@__DIR__, "..", "..", "common", "runner.jl"))

function test_runner()
    Test.@testset "LocalTestRunner" verbose = VERBOSE showtiming = SHOWTIMING begin
        R = LocalTestRunner

        Test.@testset "selection" begin
            Test.@test R.selected("test_runner.jl", String[])
            Test.@test R.selected("test_runner.jl", ["runner"])
            Test.@test R.selected("test_runner.jl", ["test_runner"])
            Test.@test R.selected("test_runner.jl", ["test_runner.jl"])
            Test.@test R.selected("test_runner.jl", ["*run*"])
            Test.@test R.selected("test_runner.jl", ["nothing", "runner"])
            Test.@test !R.selected("test_runner.jl", ["aqua"])
            # `.` is literal in a pattern, not "any character"
            Test.@test R.selected("test_a.b.jl", ["test_a.b"])
            Test.@test !R.selected("test_aXb.jl", ["test_a.b"])
        end

        Test.@testset "discovery and run" begin
            mktempdir() do dir
                write(joinpath(dir, "test_b.jl"), "test_b() = nothing")
                write(joinpath(dir, "test_a.jl"), "test_a() = nothing")
                write(joinpath(dir, "helper.jl"), "")

                Test.@test R.test_files(dir) == ["test_a.jl", "test_b.jl"]
                Test.@test R.test_files(joinpath(dir, "missing")) == String[]

                Test.@test R.run_dir(dir)
                Test.@test R.run_dir(dir; patterns=["a"])
                # nothing selected: reported, not an error
                Test.@test !R.run_dir(dir; patterns=["zzz"])
                Test.@test !R.run_dir(joinpath(dir, "missing"))
            end
        end
    end
end

end # module TestRunner

test_runner() = TestRunner.test_runner()
