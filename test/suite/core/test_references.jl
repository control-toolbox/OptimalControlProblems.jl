module TestReferences

using Test: Test
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

const OCP = OptimalControlProblems
const TOOL = joinpath(
    pkgdir(OptimalControlProblems), "tools", "references", "references.jl"
)
isdefined(@__MODULE__, :ReferenceObjectives) || include(TOOL)

# Fake problems: the tool is tested on its own, apart from the collection.
const DEFAULTS = Dict(
    :alpha => (grid_size=100, parameters="(a = 1,)"),
    :omega => (grid_size=50, parameters="nothing"),
)
const HEADER = "test\tproblem\tmodeling\tconfiguration\tgrid_size\tscheme\tresult\treason\tobjective\titerations\ttime\tgap"

function row(test, problem, modeling, grid, result, objective)
    return join(
        (
            test,
            problem,
            modeling,
            "c=1",
            grid,
            "trapeze",
            result,
            "R",
            objective,
            "3",
            "0.1",
            "",
        ),
        "\t",
    )
end

function results_dir()
    dir = mktempdir()
    mkpath(joinpath(dir, "cron-ubuntu"))
    write(
        joinpath(dir, "cron-ubuntu", "X-cron-results.tsv"),
        join(
            [
                HEADER,
                row(
                    "test_convergence c=1", "alpha", "A", "100", "converged", "2.0000000001"
                ),
                row("test_convergence c=1", "alpha", "B", "100", "converged", "2.0"),
                row("test_convergence c=1", "alpha", "C", "100", "not_converged", "1.0"), # not converged
                row("test_convergence c=1", "alpha", "D", "10", "converged", "1.5"), # not the default grid
                row("test_smoke c=1", "alpha", "E", "100", "converged", "1.0"), # not a convergence test
                row("test_convergence c=1", "omega", "A", "50", "converged", "3.0"),
                row("test_convergence c=1", "omega", "B", "50", "converged", "3.3"),
                row("test_convergence c=1", "gamma", "A", "50", "converged", "1.0"), # unknown problem
            ],
            "\n",
        ) * "\n",
    )
    write(
        joinpath(dir, "local-results.tsv"),
        HEADER *
        "\n" *
        row("test_convergence c=1", "omega", "C", "50", "converged", "3.0") *
        "\n",
    )
    return dir
end

function test_references()
    Test.@testset "reference objectives" verbose = VERBOSE showtiming = SHOWTIMING begin
        R = ReferenceObjectives
        rows = R.read_results([results_dir()])
        Test.@test length(rows) == 9
        Test.@test count(r -> r["platform"] == "cron-ubuntu", rows) == 8
        Test.@test count(r -> r["platform"] == "local", rows) == 1

        tables = R.references(rows, DEFAULTS; run="https://run")
        Test.@test sort!(collect(keys(tables))) == [:alpha, :omega]
        alpha = tables[:alpha]["trapeze"]
        # the best converged objective at the default grid, from convergence tests only
        Test.@test alpha["value"] == 2.0
        Test.@test length(alpha["results"]) == 2
        Test.@test alpha["agreement"]
        Test.@test alpha["grid_size"] == 100 && alpha["parameters"] == "(a = 1,)"
        omega = tables[:omega]["trapeze"]
        Test.@test omega["value"] == 3.0
        Test.@test omega["spread"] ≈ 0.1
        Test.@test !omega["agreement"]
        Test.@test any(contains("configurations disagree"), R.summary(tables))

        # written, then read back by the package as its metadata reads it
        dir = mktempdir()
        files = R.write_references(dir, tables)
        Test.@test basename.(files) == ["alpha.toml", "omega.toml"]
        Test.@test startswith(read(files[1], String), "# Reference objectives")
        read_back = OCP._references(:alpha, files[1])
        Test.@test read_back.trapeze.value == 2.0
        Test.@test read_back.trapeze.source === :computed
        Test.@test read_back.trapeze.run == "https://run"
        # a scheme not recomputed is kept
        R.write_references(dir, Dict(:alpha => Dict{String,Any}("midpoint" => alpha)))
        Test.@test keys(OCP._references(:alpha, files[1])) ⊇ (:trapeze, :midpoint)

        # a file that misses a field is an error at load time
        broken = joinpath(dir, "broken.toml")
        write(broken, "[trapeze]\nvalue = 1.0\n")
        Test.@test_throws ErrorException OCP._references(:alpha, broken)
        Test.@test isnothing(OCP._references(:alpha, joinpath(dir, "none.toml")))
    end
end

end # module TestReferences

test_references() = TestReferences.test_references()
