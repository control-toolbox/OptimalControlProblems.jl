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
            Test.@test R.split_mode(["robustness", "OptimalControl"]) ==
                (:robustness, ["OptimalControl"])
            Test.@test_throws ArgumentError R.split_mode(["cron", "convergence"])
            Test.@test_throws ArgumentError R.split_mode(["cron", "robustness"])
            mktempdir() do root
                suite, cron = joinpath(root, "suite"), joinpath(root, "cron")
                robustness = joinpath(root, "robustness")
                mkpath(joinpath(suite, "a"))
                mkpath(joinpath(cron, "a"))
                mkpath(joinpath(suite, "b"))
                mkpath(joinpath(robustness, "a"))
                dirs(group, mode) = R.directories(group, suite, cron, robustness; mode=mode)
                Test.@test dirs("a", :suite) == [joinpath(suite, "a")]
                Test.@test dirs("a", :cron) == [joinpath(cron, "a")]
                Test.@test dirs("a", :convergence) ==
                    [joinpath(suite, "a"), joinpath(cron, "a")]
                # the robustness directories run only when asked for, alone
                Test.@test dirs("a", :robustness) == [joinpath(robustness, "a")]
                Test.@test isempty(dirs("b", :robustness))
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

        Test.@testset "budget (#435)" begin
            Test.@test R.split_budget(["JuMP", "problem=a"]) ==
                (nothing, ["JuMP", "problem=a"])
            Test.@test R.split_budget(["budget=3600", "cron"]) ==
                ((seconds=3600.0, timeout=R.DEFAULT_TIMEOUT), ["cron"])
            Test.@test R.split_budget(["timeout=60", "budget=600"]) ==
                ((seconds=600.0, timeout=60.0), String[])
            # a timeout alone changes nothing: the rotation needs a budget
            Test.@test R.split_budget(["timeout=60"]) == (nothing, String[])
            Test.@test_throws ArgumentError R.split_budget(["budget=0"])
            Test.@test_throws ArgumentError R.split_budget(["budget=one"])

            # a clock that advances 10 s per problem: the budget is checked before each one
            now = Ref(0.0)
            clock() = now[]
            ran = String[]
            outcomes, skipped = R.run_within_budget(["a", "b", "c", "d"], 25; clock) do p
                push!(ran, p)
                now[] += 10
                return :success
            end
            Test.@test ran == ["a", "b", "c"]
            Test.@test outcomes == ["a" => :success, "b" => :success, "c" => :success]
            Test.@test skipped == ["d"]
            # the first problem always runs, even past the budget
            now[] = 0.0
            outcomes, skipped = R.run_within_budget(["a", "b"], 5; clock) do p
                now[] += 10
                return :failed
            end
            Test.@test outcomes == ["a" => :failed]
            Test.@test skipped == ["b"]

            julia = Base.julia_cmd()
            Test.@test R.run_process(`$julia --startup-file=no -e 0`; timeout=60) ===
                :success
            Test.@test R.run_process(
                `$julia --startup-file=no -e "exit(1)"`; timeout=60
            ) === :failed
            Test.@test R.run_process(
                `$julia --startup-file=no -e "sleep(60)"`; timeout=2
            ) === :timeout
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
