module TestRunner

using Test: Test

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

# The selection is a plain file shared by the dispatcher and every environment.
include(joinpath(@__DIR__, "..", "..", "common", "selection.jl"))

function test_runner()
    Test.@testset "TestSelection" verbose = VERBOSE showtiming = SHOWTIMING begin
        R = TestSelection

        Test.@testset "environment selection" begin
            avail = ["JuMP", "ModelingToolkit", "OptimalControl"]
            opt_in = ("ModelingToolkit",)
            sel(args) = R.select_environments(args, avail, opt_in)
            # nothing named: every environment but the opt-in ones
            Test.@test sel(String[]) == (["JuMP", "OptimalControl"], String[])
            Test.@test sel(["*kwargs*"]) == (["JuMP", "OptimalControl"], ["*kwargs*"])
            # named: only those, patterns apart
            Test.@test sel(["JuMP"]) == (["JuMP"], String[])
            Test.@test sel(["ModelingToolkit", "*solve*"]) ==
                (["ModelingToolkit"], ["*solve*"])
            # `core` alone: no other environment (it always runs in-process)
            Test.@test sel(["core"]) == (String[], String[])
            Test.@test sel(["core", "JuMP"]) == (["JuMP"], String[])
        end

        Test.@testset "mode" begin
            Test.@test R.split_mode(String[]) == (:suite, String[])
            Test.@test R.split_mode(["JuMP", "cron"]) == (:cron, ["JuMP"])
            Test.@test R.split_mode(["convergence", "*solve*"]) ==
                (:convergence, ["*solve*"])
            Test.@test_throws ArgumentError R.split_mode(["cron", "convergence"])
            mktempdir() do root
                suite, cron = joinpath(root, "suite"), joinpath(root, "cron")
                mkpath(joinpath(suite, "a"))
                mkpath(joinpath(cron, "a"))
                mkpath(joinpath(suite, "b"))
                dirs(group, mode) = R.directories(group, suite, cron; mode=mode)
                Test.@test dirs("a", :suite) == [joinpath(suite, "a")]
                Test.@test dirs("a", :cron) == [joinpath(cron, "a")]
                Test.@test dirs("a", :convergence) ==
                    [joinpath(suite, "a"), joinpath(cron, "a")]
                # no cron directory: nothing in cron mode, the suite only for convergence
                Test.@test isempty(dirs("b", :cron))
                Test.@test dirs("b", :convergence) == [joinpath(suite, "b")]
            end
        end

        Test.@testset "problem selection" begin
            Test.@test R.split_problems(String[]) == (String[], String[])
            Test.@test R.split_problems(["JuMP", "problem=a,b", "*solve*"]) ==
                (["a", "b"], ["JuMP", "*solve*"])
            # several arguments add up, duplicates and empty names go
            Test.@test R.split_problems(["problem=a,,b", "problem=b,c"]) ==
                (["a", "b", "c"], String[])
        end

        Test.@testset "run" begin
            RAN = Main.TEST_RAN
            mktempdir() do dir
                write(joinpath(dir, "test_b.jl"), "test_b() = push!(Main.TEST_RAN, :b)")
                write(joinpath(dir, "test_a.jl"), "test_a() = push!(Main.TEST_RAN, :a)")
                empty!(RAN)

                # `CTBase.DevTools.run_tests` discovers, selects (patterns as before) and
                # runs the entry function of each file
                Test.@test R.run_dir(dir; name="d")
                Test.@test sort(RAN) == [:a, :b]
                empty!(RAN)
                Test.@test R.run_dir(dir; patterns=["a"], name="d")
                Test.@test RAN == [:a]
                empty!(RAN)
                Test.@test R.run_dir(dir; patterns=["*b*", "zzz"], name="d")
                Test.@test RAN == [:b]
                empty!(RAN)
                # nothing selected: nothing runs, no error
                Test.@test R.run_dir(dir; patterns=["zzz"], name="d")
                Test.@test isempty(RAN)
                # no directory (`core` in cron mode): reported by the return value
                Test.@test !R.run_dir(joinpath(dir, "missing"))
            end
        end
    end
end

end # module TestRunner

# Filled by the files that the test runs: they are included in `Main`.
const TEST_RAN = Symbol[]

test_runner() = TestRunner.test_runner()
