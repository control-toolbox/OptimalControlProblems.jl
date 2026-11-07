module TestStatusLoop

using TOML: TOML
using Test: Test
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

const BACKEND = OptimalControlProblems.JuMPBackend()

"""Run `f` with the problem selection `names`, then restore the previous one."""
function with_selection(f, names)
    previous = Main.TestConfigs.PROBLEMS[]
    try
        Main.TestStatus.select_problems!(names)
        return f()
    finally
        Main.TestConfigs.PROBLEMS[] = previous
    end
end

"""Problems visited by `for_each_problem` (the test bodies do nothing)."""
function visited()
    seen = Symbol[]
    Main.TestStatus.for_each_problem(problem -> push!(seen, problem), "visited", BACKEND)
    return seen
end

function test_status()
    Test.@testset "status loop and problem selection" verbose = VERBOSE showtiming =
        SHOWTIMING begin
        all_problems = OptimalControlProblems.problems(BACKEND; status=:all)
        # whatever the selection of this run (`problem=…`), the loop without selection visits all
        Test.@test with_selection(visited, String[]) == all_problems

        Test.@testset "selection" begin
            one = first(all_problems)
            Test.@test with_selection(visited, [string(one)]) == [one]
            # a selection restricts, it does not extend: a problem absent from the modeling
            # is not visited
            absent = setdiff(OptimalControlProblems.problems(; status=:all), all_problems)
            isempty(absent) ||
                Test.@test with_selection(visited, [string(first(absent))]) == Symbol[]
            # an unknown name is an error: a selection matching nothing would test nothing
            Test.@test_throws ArgumentError Main.TestStatus.select_problems!([
                "not_a_problem"
            ])
            # no name: no restriction
            Test.@test with_selection(visited, String[]) == all_problems
        end

        Test.@testset "drafts do not fail the run" begin
            # a draft problem whose test fails is recorded, not reported to the parent
            # (`@testset` takes a name, not a qualified one, on Julia 1.10)
            DraftTestSet = Main.TestStatus.DraftTestSet
            Test.@test DraftTestSet("draft").fail == 0
            ts = Test.@testset DraftTestSet "inner" begin
                Test.@test false
                Test.@test true
            end
            Test.@test (ts.pass, ts.fail) == (1, 1)
            Test.@test Main.TestStatus.outcome(ts) == "fail"
        end

        Test.@testset "nested valid outcomes" begin
            passing = Test.DefaultTestSet("outer")
            inner_pass = Test.DefaultTestSet("inner")
            Test.record(inner_pass, Test.Pass(:test, nothing, nothing, nothing))
            Test.record(passing, inner_pass)
            Test.@test Main.TestStatus.outcome(passing) == "pass"

            failing = Test.DefaultTestSet("outer")
            inner_fail = Test.DefaultTestSet("inner")
            Test.record(
                inner_fail, Test.Fail(:test, nothing, nothing, nothing, LineNumberNode(1))
            )
            Test.record(failing, inner_fail)
            Test.@test Main.TestStatus.outcome(failing) == "fail"
        end

        Test.@testset "how a solve was configured is recorded (#409)" begin
            S = Main.TestStatus
            # the options are read back with TOML: an enumeration value and a symbol as
            # their names, numbers as they are, the non-finite ones too
            options = (
                tol=1e-8,
                max_iter=1000,
                sb="yes",
                barrier=:loqo,
                print_level=Base.Order.Forward,
                flag=true,
                wall=Inf,
                nan=NaN,
            )
            back = TOML.parse("o = " * S.inline_toml(options))["o"]
            Test.@test back["tol"] === 1e-8
            Test.@test back["max_iter"] === 1000
            Test.@test back["sb"] == "yes"
            Test.@test back["barrier"] == "loqo"
            Test.@test back["print_level"] == string(Base.Order.Forward)
            Test.@test back["flag"] === true
            Test.@test back["wall"] == Inf
            Test.@test isnan(back["nan"])

            # a result has its settings and its violation in the results file, after the
            # columns the cron report reads
            problem = first(OptimalControlProblems.problems(BACKEND; status=:all))
            n = length(S.RESULTS)
            tsv = tempname() * "-results.tsv"
            try
                S.result!(
                    "test_convergence solver=ipopt",
                    problem,
                    BACKEND,
                    "solver=ipopt";
                    modeler=nothing,
                    solver=:ipopt,
                    options=(tol=1e-8, mu_init=1e-3),
                    grid_size=OptimalControlProblems.grid_size_data(problem),
                    scheme=:trapeze,
                    converged=true,
                    reason="LOCALLY_SOLVED",
                    objective=1.0,
                    iterations=7,
                    violation=2.5e-9,
                    time=0.1,
                )
                S.results_table(tsv)
                header, row = split.(readlines(tsv)[[1, end]], '\t')
                cell(name) = row[findfirst(==(name), header)]
                Test.@test header[(end - 3):end] ==
                    ["modeler", "solver", "options", "violation"]
                Test.@test cell("modeler") == ""
                Test.@test cell("solver") == "ipopt"
                Test.@test TOML.parse("o = " * cell("options"))["o"] ==
                    Dict("tol" => 1e-8, "mu_init" => 1e-3)
                Test.@test parse(Float64, cell("violation")) == 2.5e-9
            finally
                resize!(S.RESULTS, n)
                rm(tsv; force=true)
            end

            # the environment of the run: Julia, the machine, the commit, the run, and the
            # versions of the packages
            file = tempname() * "-environment.toml"
            try
                env = TOML.parsefile(S.write_environment(file))
                Test.@test env["julia"] == string(VERSION)
                for key in ("os", "arch", "cpu", "commit", "run")
                    Test.@test !isempty(env[key])
                end
                Test.@test haskey(env["packages"], "Test")
            finally
                rm(file; force=true)
            end
        end
    end
end

end # module TestStatusLoop

test_status() = TestStatusLoop.test_status()
