module TestModules

using Test: Test
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

const O = OptimalControlProblems
const SRC = joinpath(pkgdir(O), "src")

"""The submodules of `m`, read from the package: no list is written here."""
function submodules(m::Module)
    return [
        x for x in (getfield(m, n) for n in names(m; all=true)) if
        x isa Module && x !== m && parentmodule(x) === m
    ]
end

"""
The module where the binding `name` of the package is defined: a submodule for every name
the package re-exports (the aliases of CTModels functions included, which `parentmodule`
would place in CTModels).
"""
owner(name::Symbol) = Base.binding_module(O, name)

# The layout of the source (#413, ct-conventions MOD1, MOD8 as adapted for a package used
# directly): one submodule per directory `src/<Name>/<Name>.jl`, the package re-exporting
# their public API.
function test_modules()
    Test.@testset "submodules and exports" verbose = VERBOSE showtiming = SHOWTIMING begin
        modules = submodules(O)
        Test.@test !isempty(modules)

        Test.@testset "one directory per submodule (MOD1)" begin
            directories = sort!(filter(d -> isdir(joinpath(SRC, d)), readdir(SRC)))
            Test.@test directories == sort!(string.(nameof.(modules)))
            for m in modules
                name = string(nameof(m))
                Test.@test isfile(joinpath(SRC, name, "$(name).jl"))
            end
        end

        Test.@testset "the package defines nothing, it re-exports" begin
            # every exported name of the package is a binding of one submodule: the
            # package file only declares (MOD2)
            for name in names(O)
                name === nameof(O) && continue
                Test.@test owner(name) in modules
            end
            # every problem has its function, exported
            for problem in O.Metadata.PROBLEM_NAMES
                Test.@test problem in names(O)
                Test.@test parentmodule(getfield(O, problem)) === O.Catalogue
            end
        end

        Test.@testset "the public API of a submodule is reachable from the package" begin
            # `OptimalControlProblems.status(problem)`, `OptimalControlProblems.features(…)`:
            # what a submodule exports is called from the package, exported or not
            for m in modules, name in names(m)
                name === nameof(m) && continue
                Test.@test isdefined(O, name)
                Test.@test getfield(O, name) === getfield(m, name)
            end
        end
    end
end

end # module TestModules

test_modules() = TestModules.test_modules()
