module TestReferences

using CTBase: CTBase
using Test: Test
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

const OCP = OptimalControlProblems
const TOOL = joinpath(
    pkgdir(OptimalControlProblems), "tools", "references", "references.jl"
)
isdefined(@__MODULE__, :ReferenceObjectives) || include(TOOL)

# A reference type that does not implement the contract (TST3: at the top level).
struct FakeReference <: OptimalControlProblems.AbstractReference end

# Fake problems: the tool is tested on its own, apart from the collection. The grid of a
# problem depends on the modeling (`grid_size_data(problem, backend)`).
const DEFAULTS = Dict(
    :alpha => (
        grid_sizes=Dict("JuMP" => 100, "OptimalControl" => 100, "ModelingToolkit" => 20),
        parameters="(a = 1,)",
    ),
    :omega => (grid_sizes=Dict("JuMP" => 50), parameters="nothing"),
)
const HEADER = "test\tproblem\tmodeling\tconfiguration\tgrid_size\tscheme\tresult\treason\tobjective\titerations\ttime\tgap\tmodeler\tsolver\toptions\tviolation"

# A row of a results file; `reason` and `violation` choose the representative of a solution
# (#409): a strict convergence before an acceptable one, then the smallest violation.
function row(
    test,
    problem,
    modeling,
    grid,
    result,
    objective;
    reason="R",
    violation="1.0e-9",
    configuration="c=1",
)
    return join(
        (
            test,
            problem,
            modeling,
            configuration,
            grid,
            "trapeze",
            result,
            reason,
            objective,
            "3",
            "0.1",
            "",
            "adnlp",
            "ipopt",
            "{tol = 1.0e-8, mu_strategy = \"adaptive\"}",
            violation,
        ),
        "\t",
    )
end

# The environment of a run, written next to its results file by the tests (#412).
function environment(file; run="https://run/1")
    return write(
        replace(file, "-results.tsv" => "-environment.toml"),
        """
        arch = "x86_64"
        commit = "abc"
        cpu = "CPU"
        julia = "1.13.3"
        os = "Linux"
        run = "$(run)"
        threads = 1

        [packages]
        Ipopt_jll = "300.1400.1902+0"
        """,
    )
end

function results_dir()
    dir = mktempdir()
    mkpath(joinpath(dir, "cron-ubuntu"))
    file = joinpath(dir, "cron-ubuntu", "X-cron-results.tsv")
    environment(file)
    write(
        file,
        join(
            [
                HEADER,
                # the smallest objective and violation, but acceptable: not the representative
                row(
                    "test_convergence c=1",
                    "alpha",
                    "JuMP",
                    "100",
                    "converged",
                    "1.99999999";
                    reason="acceptable",
                    violation="1.0e-13",
                ),
                row(
                    "test_convergence c=1",
                    "alpha",
                    "JuMP",
                    "100",
                    "converged",
                    "2.0";
                    violation="1.0e-6",
                    configuration="c=2",
                ),
                # strict, the smallest violation: the representative of the solution
                row(
                    "test_convergence c=1",
                    "alpha",
                    "JuMP",
                    "100",
                    "converged",
                    "2.0000000001";
                    violation="1.0e-12",
                    configuration="c=3",
                ),
                row("test_convergence c=1", "alpha", "JuMP", "100", "not_converged", "1.0"), # not converged
                row("test_convergence c=1", "alpha", "JuMP", "10", "converged", "1.5"), # not the default grid
                row("test_smoke c=1", "alpha", "JuMP", "100", "converged", "1.0"), # not a convergence test
                row(
                    "test_robustness c=1 mu=1",
                    "alpha",
                    "JuMP",
                    "100",
                    "converged",
                    "2.00001";
                    configuration="c=4",
                ), # same solution
                # another modeling: its own reference, whatever the others reach
                row(
                    "test_convergence c=1",
                    "alpha",
                    "OptimalControl",
                    "100",
                    "converged",
                    "2.5",
                ),
                # its own grid: ModelingToolkit on 20 steps
                row(
                    "test_convergence c=1",
                    "alpha",
                    "ModelingToolkit",
                    "20",
                    "converged",
                    "2.2",
                ),
                row(
                    "test_convergence c=1",
                    "alpha",
                    "ModelingToolkit",
                    "100",
                    "converged",
                    "9.0",
                ), # not its grid
                row("test_convergence c=1", "omega", "JuMP", "50", "converged", "3.0"),
                row(
                    "test_convergence c=1",
                    "omega",
                    "JuMP",
                    "50",
                    "converged",
                    "3.3";
                    configuration="c=2",
                ),
                row("test_convergence c=1", "gamma", "JuMP", "50", "converged", "1.0"), # unknown problem
            ],
            "\n",
        ) * "\n",
    )
    write(
        joinpath(dir, "local-results.tsv"),
        HEADER *
        "\n" *
        row(
            "test_convergence c=1",
            "omega",
            "JuMP",
            "50",
            "converged",
            "3.0";
            configuration="c=3",
        ) *
        "\n",
    )
    environment(joinpath(dir, "local-results.tsv"); run="local")
    return dir
end

function test_references()
    Test.@testset "reference objectives" verbose = VERBOSE showtiming = SHOWTIMING begin
        R = ReferenceObjectives
        rows = R.read_results([results_dir()])
        Test.@test length(rows) == 14
        Test.@test count(r -> r["platform"] == "cron-ubuntu", rows) == 13
        Test.@test count(r -> r["platform"] == "local", rows) == 1

        tables = R.references(rows, DEFAULTS; run="https://run")
        Test.@test sort!(collect(keys(tables))) == [:alpha, :omega]
        # one table per modeling and scheme (#435): the modelings are independent
        Test.@test sort!(collect(keys(tables[:alpha]))) ==
            ["JuMP", "ModelingToolkit", "OptimalControl"]
        Test.@test tables[:alpha]["OptimalControl"]["trapeze"]["value"] == 2.5
        mtk = tables[:alpha]["ModelingToolkit"]["trapeze"]
        Test.@test mtk["value"] == 2.2 && mtk["grid_size"] == 20
        alpha = tables[:alpha]["JuMP"]["trapeze"]
        # the objective of the representative of the best solution, at the default grid, from
        # convergence and robustness tests (#409): strict, then the smallest violation
        Test.@test alpha["value"] == 2.0000000001
        Test.@test length(alpha["results"]) == 4
        Test.@test all(r -> r["modeling"] == "JuMP", alpha["results"])
        # within SAME_SOLUTION (1e-5): one solution, the configurations agree
        Test.@test length(alpha["solutions"]) == 1
        Test.@test alpha["solutions"][1]["solves"] == 4
        reference = alpha["solutions"][1]["reference"]
        Test.@test reference["configuration"] == "c=3"
        Test.@test reference["objective"] == alpha["value"]
        Test.@test reference["violation"] == 1e-12
        Test.@test reference["status"] == "R"
        Test.@test reference["options"] == Dict("tol" => 1e-8, "mu_strategy" => "adaptive")
        Test.@test (reference["modeler"], reference["solver"]) == ("adnlp", "ipopt")
        Test.@test reference["run"] == "https://run/1" && reference["commit"] == "abc"
        Test.@test reference["platform"] == "cron-ubuntu"
        Test.@test reference["versions"] == Dict("Ipopt_jll" => "300.1400.1902+0")
        # without a strict convergence, an acceptable solve represents its solution
        acceptable = R.representative([
            merge(r, Dict("reason" => "acceptable")) for
            r in rows if r["problem"] == "omega"
        ])
        Test.@test acceptable["reason"] == "acceptable"
        Test.@test alpha["agreement"]
        Test.@test alpha["grid_size"] == 100 && alpha["parameters"] == "(a = 1,)"
        omega = tables[:omega]["JuMP"]["trapeze"]
        Test.@test omega["value"] == 3.0
        Test.@test omega["spread"] ≈ 0.1
        # 3.0 twice and 3.3: two solutions, best first
        Test.@test [s["objective"] for s in omega["solutions"]] == [3.0, 3.3]
        Test.@test omega["solutions"][1]["configurations"] == ["JuMP c=1", "JuMP c=3"]
        Test.@test !omega["agreement"]
        Test.@test any(contains("omega JuMP trapeze"), R.summary(tables))
        Test.@test any(contains("2 solutions"), R.summary(tables))

        # written, then read back by the package as its metadata reads it
        dir = mktempdir()
        files = R.write_references(dir, tables)
        Test.@test basename.(files) == ["alpha.toml", "omega.toml"]
        Test.@test startswith(read(files[1], String), "# Reference objectives")
        read_back = OCP.References._references(:alpha, files[1])
        Test.@test read_back.JuMP.trapeze.value == 2.0000000001
        Test.@test read_back.OptimalControl.trapeze.value == 2.5
        Test.@test read_back.ModelingToolkit.trapeze.grid_size == 20
        Test.@test read_back.JuMP.trapeze.source === :computed
        Test.@test read_back.JuMP.trapeze.run == "https://run"
        Test.@test read_back.JuMP.trapeze.solutions == [2.0000000001]
        # the known solution with how it was obtained (#409)
        ref = only(read_back.JuMP.trapeze.references)
        Test.@test ref isa OCP.NumericalReference
        Test.@test (ref.problem, ref.scheme, ref.grid_size) == (:alpha, :trapeze, 100)
        Test.@test OCP.objective(ref) == 2.0000000001
        Test.@test (ref.modeling, ref.modeler, ref.solver, ref.status) ==
            (:JuMP, :adnlp, :ipopt, :R)
        Test.@test ref.options == (mu_strategy="adaptive", tol=1e-8)
        Test.@test ref.julia == v"1.13.3" && ref.machine.os == "Linux"
        Test.@test ref.versions.Ipopt_jll == "300.1400.1902+0"
        Test.@test ref.solves == 4 && ref.violation == 1e-12
        # shown in one line, and in full with what says how it was obtained
        Test.@test sprint(show, ref) == "NumericalReference(:alpha, 2.0000000001)"
        shown = sprint(show, MIME("text/plain"), ref)
        Test.@test startswith(shown, "Known solution of alpha, trapeze on 100 steps")
        Test.@test occursin("solved by:  JuMP, adnlp + ipopt, R", shown)
        Test.@test occursin("run:        https://run/1 (cron-ubuntu)", shown)
        Test.@test OCP.References._references(:omega, files[2]).JuMP.trapeze.solutions ==
            [3.0, 3.3]
        # a table not recomputed is kept: another scheme, another modeling
        R.write_references(
            dir, Dict(:alpha => Dict{String,Any}("JuMP" => Dict("midpoint" => alpha)))
        )
        again = OCP.References._references(:alpha, files[1])
        Test.@test keys(again.JuMP) ⊇ (:trapeze, :midpoint)
        Test.@test again.OptimalControl.trapeze.value == 2.5

        # a file that misses a field is an error at load time
        broken = joinpath(dir, "broken.toml")
        write(broken, "[JuMP.trapeze]\nvalue = 1.0\n")
        Test.@test_throws CTBase.Exceptions.IncorrectArgument OCP.References._references(
            :alpha, broken
        )
        # a table that is not a modeling (a file written before #435) is an error
        unknown = joinpath(dir, "unknown.toml")
        write(unknown, "[trapeze]\nvalue = 1.0\n")
        Test.@test_throws CTBase.Exceptions.IncorrectArgument OCP.References._references(
            :alpha, unknown
        )
        Test.@test isnothing(OCP.References._references(:alpha, joinpath(dir, "none.toml")))

        # a file written before #409 loads, without references, so that the tool can write
        # it again; the consistency tests fail on it
        old = joinpath(dir, "old.toml")
        table = deepcopy(alpha)
        foreach(s -> delete!(s, "reference"), table["solutions"])
        R.write_references(
            dir, Dict(:old => Dict{String,Any}("JuMP" => Dict("trapeze" => table)))
        )
        Test.@test isempty(OCP.References._references(:old, old).JuMP.trapeze.references)

        # results recorded before #409, or without their environment, cannot say how a
        # solution was obtained: an error
        stale = mktempdir()
        write(
            joinpath(stale, "X-results.tsv"), join(split(HEADER, '\t')[1:12], '\t') * "\n"
        )
        Test.@test_throws ErrorException R.read_results([stale])
        bare = mktempdir()
        write(joinpath(bare, "X-results.tsv"), HEADER * "\n")
        Test.@test_throws ErrorException R.read_results([bare])
    end
end

# The references of the collection as values (#409): known solutions and analytic ones.
function test_reference_api()
    Test.@testset "reference values" verbose = VERBOSE showtiming = SHOWTIMING begin
        Test.@test_throws CTBase.Exceptions.NotImplemented OCP.objective(FakeReference())
        backends = [spec.backend() for spec in OCP.Modelings.MODELING_SPECS]
        for problem in OCP.problems(; status=:all)
            for backend in backends
                found = OCP.references(problem, backend)
                best = OCP.best_reference(problem, backend)
                name = OCP.Modelings._modeling_spec(backend).name
                if isempty(found)
                    Test.@test isnothing(best)
                    Test.@test isnothing(OCP.reference_objective(problem, backend))
                else
                    Test.@test best === first(found)
                    Test.@test OCP.objective(best) ==
                        OCP.reference_objective(problem, backend)
                    Test.@test issorted(OCP.objective.(found))
                    Test.@test all(r -> r.problem === problem, found)
                    # the references of a modeling come from its solves only (#435)
                    Test.@test all(r -> r.modeling === name, found)
                    Test.@test all(
                        r -> OCP.solution_kind(problem, backend, OCP.objective(r)) in
                        (:best, :local),
                        found,
                    )
                end
                # another discretization has no reference
                Test.@test isempty(OCP.references(problem, backend; grid_size=1))
            end
            analytic = OCP.analytic_reference(problem)
            data = OCP.reference_solution_data(problem)
            Test.@test isnothing(analytic) == isnothing(data)
            isnothing(analytic) && continue
            Test.@test analytic isa OCP.AnalyticReference
            Test.@test sprint(show, analytic) ==
                "AnalyticReference(:$(problem), $(OCP.objective(analytic)))"
            Test.@test OCP.objective(analytic) == data.objective
            Test.@test analytic.source == data.source
        end
    end
end

end # module TestReferences

test_references() = (TestReferences.test_references(); TestReferences.test_reference_api())
