module TestDiscoveryCache

using Pkg: Pkg
using Test: Test
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

function copy_package()
    package = mktempdir()
    root = pkgdir(OptimalControlProblems)
    cp(joinpath(root, "Project.toml"), joinpath(package, "Project.toml"))
    cp(joinpath(root, "src"), joinpath(package, "src"))
    cp(joinpath(root, "ext"), joinpath(package, "ext"))
    return package
end

function julia_process(depot)
    separator = Sys.iswindows() ? ';' : ':'
    path = join(vcat([depot], DEPOT_PATH), separator)
    return addenv(
        Base.julia_cmd(),
        "JULIA_LOAD_PATH" => nothing,
        "JULIA_PROJECT" => nothing,
        "JULIA_DEPOT_PATH" => path,
    )
end

function run_package(julia, package, code)
    process = run(`$julia --startup-file=no --project=$package -e $code`; wait=false)
    wait(process)
    return process
end

function cache_files(depot)
    cache_dir = joinpath(
        depot, "compiled", "v$(VERSION.major).$(VERSION.minor)", "OptimalControlProblems"
    )
    isdir(cache_dir) || return NamedTuple[]
    return [
        (path=path, mtime=stat(path).mtime, size=filesize(path)) for
        path in sort(filter(endswith(".ji"), readdir(cache_dir; join=true)))
    ]
end

function query_fixture(julia, package)
    output = read(
        `$julia --startup-file=no --project=$package -e 'using OptimalControlProblems; println(:foo in problems(JuMPBackend(); status=:all))'`,
        String,
    )
    return strip(output) == "true"
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

# Remove `pattern` from the file `parts` of `package`; it must be there.
function remove_line(package, parts, pattern)
    path = joinpath(package, parts...)
    source = read(path, String)
    @assert occursin(pattern, source)
    return write(path, replace(source, pattern => ""))
end

function remove_include_dependencies(package)
    # The checkout has CRLF line endings on Windows: `\r?\n`.
    remove_line(
        package,
        ("src", "Metadata", "discovery.jl"),
        r"    Base\.include_dependency\(path\)\r?\n",
    )
    return remove_line(
        package,
        ("src", "Modelings", "backends.jl"),
        r"    Base\.include_dependency\(joinpath\(EXT_PATH, spec\.directory\)\)\r?\n",
    )
end

function cleanup(path)
    try
        rm(path; recursive=true, force=true)
    catch exception
        @warn "Could not remove temporary test directory" path exception
    end
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
            Test.@test success(
                run_package(
                    julia,
                    package,
                    setup *
                    "; @assert :foo ∉ OptimalControlProblems.problems(; status=:all)",
                ),
            )

            add_fixture(package, problem)
            Test.@test success(
                run_package(
                    julia,
                    package,
                    """
                    using OptimalControlProblems
                    @assert :foo in problems(JuMPBackend(); status=:draft)
                    @assert :foo ∉ problems(JuMPBackend())
                    @assert :foo ∉ problems(OptimalControlBackend(); status=:all)
                    """,
                ),
            )

            set_fixture_status(package, :valid)
            Test.@test success(
                run_package(
                    julia,
                    package,
                    "using OptimalControlProblems; @assert :foo in problems(JuMPBackend())",
                ),
            )

            rm(joinpath(package, "ext", "JuMPModels", "foo.jl"))
            Test.@test success(
                run_package(
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
                ),
            )

            rm(joinpath(package, "ext", "MetaData", "foo.jl"))
            remove_include_dependencies(package)
            Test.@test success(
                run_package(
                    julia,
                    package,
                    "using OptimalControlProblems; @assert :foo ∉ problems(JuMPBackend(); status=:all)",
                ),
            )
            before = cache_files(depot)
            add_fixture(package, problem)
            foo_seen = query_fixture(julia, package)
            after = cache_files(depot)
            reused = before == after
            @info "Precompilation cache control" before after reused
            if !reused
                Test.@test_skip false
            else
                Test.@test !foo_seen
            end
        finally
            cleanup(depot)
            cleanup(package)
        end
    end
end

end # module TestDiscoveryCache

test_discovery_cache() = TestDiscoveryCache.test_discovery_cache()
