module TestDiscoveryCache

using Pkg: Pkg
using Test: Test
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

function copy_package()
    package = mktempdir()
    root = dirname(dirname(dirname(@__DIR__)))
    cp(joinpath(root, "Project.toml"), joinpath(package, "Project.toml"))
    cp(joinpath(root, "src"), joinpath(package, "src"))
    cp(joinpath(root, "ext"), joinpath(package, "ext"))
    return package
end

function julia_process(depot)
    return addenv(
        Base.julia_cmd(),
        "JULIA_LOAD_PATH" => nothing,
        "JULIA_PROJECT" => nothing,
        "JULIA_DEPOT_PATH" => depot,
    )
end

function run_package(julia, package, code)
    return run(`$julia --startup-file=no --project=$package -e $code`)
end

function add_fixture(package, problem)
    metadata_path = joinpath(package, "ext", "MetaData", "$(problem).jl")
    metadata = read(metadata_path, String)
    metadata = replace(metadata, "$(problem)_meta" => "foo_meta"; count=1)
    metadata = replace(
        metadata, r"    :status => .*\n" => "    :status => (JuMP=:draft,),\n"; count=1
    )
    write(joinpath(package, "ext", "MetaData", "foo.jl"), metadata)

    model_path = joinpath(package, "ext", "JuMPModels", "$(problem).jl")
    model = replace(read(model_path, String), ".$problem(" => ".foo("; count=1)
    return write(joinpath(package, "ext", "JuMPModels", "foo.jl"), model)
end

function set_fixture_status(package, status)
    path = joinpath(package, "ext", "MetaData", "foo.jl")
    metadata = read(path, String)
    metadata = replace(
        metadata, r"    :status => .*\n" => "    :status => (JuMP=:$(status),),\n"; count=1
    )
    return write(path, metadata)
end

function remove_include_dependencies(package)
    path = joinpath(package, "src", "OptimalControlProblems.jl")
    source = read(path, String)
    source = replace(source, "    Base.include_dependency(path)\n" => "")
    source = replace(
        source,
        r"    Base\.include_dependency\(joinpath\(dirname\(@__FILE__\), \"\.\.\", \"ext\", spec\.directory\)\)\n" => "";
    )
    return write(path, source)
end

function test_discovery_cache()
    Test.@testset "automatic discovery and precompilation cache" verbose = VERBOSE showtiming =
        SHOWTIMING begin
        package = copy_package()
        depot = mktempdir()
        try
            julia = julia_process(depot)
            problem = first(
                OptimalControlProblems.problems(OptimalControlProblems.JuMPBackend())
            )
            setup = "using Pkg: Pkg; Pkg.instantiate(); using OptimalControlProblems"
            Test.@test run_package(
                julia,
                package,
                setup * "; @assert :foo ∉ OptimalControlProblems.problems(; status=:all)",
            ).exitcode == 0

            add_fixture(package, problem)
            Test.@test run_package(
                julia,
                package,
                """
                using OptimalControlProblems
                @assert :foo in problems(JuMPBackend(); status=:draft)
                @assert :foo ∉ problems(JuMPBackend())
                @assert :foo ∉ problems(OptimalControlBackend(); status=:all)
                """,
            ).exitcode == 0

            set_fixture_status(package, :valid)
            Test.@test run_package(
                julia,
                package,
                "using OptimalControlProblems; @assert :foo in problems(JuMPBackend())",
            ).exitcode == 0

            rm(joinpath(package, "ext", "JuMPModels", "foo.jl"))
            Test.@test run_package(
                julia,
                package,
                """
                using CTBase
                using OptimalControlProblems
                @assert :foo ∉ problems(JuMPBackend(); status=:all)
                try
                    foo(JuMPBackend())
                    error("missing modeling did not throw")
                catch exception
                    @assert exception isa CTBase.IncorrectArgument
                end
                """,
            ).exitcode == 0

            rm(joinpath(package, "ext", "MetaData", "foo.jl"))
            remove_include_dependencies(package)
            Test.@test run_package(
                julia,
                package,
                "using OptimalControlProblems; @assert :foo ∉ problems(JuMPBackend(); status=:all)",
            ).exitcode == 0
            add_fixture(package, problem)
            Test.@test run_package(
                julia,
                package,
                "using OptimalControlProblems; @assert :foo ∉ problems(JuMPBackend(); status=:all)",
            ).exitcode == 0
        finally
            rm(depot; recursive=true, force=true)
            rm(package; recursive=true, force=true)
        end
    end
end

end # module TestDiscoveryCache

test_discovery_cache() = TestDiscoveryCache.test_discovery_cache()
