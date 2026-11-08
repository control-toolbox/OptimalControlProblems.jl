"""
$(TYPEDEF)

Abstract tag selecting an optimal control problem modeling.
"""
abstract type AbstractModelBackend end

"""Select the JuMP modeling."""
struct JuMPBackend <: AbstractModelBackend end

"""Select the OptimalControl modeling."""
struct OptimalControlBackend <: AbstractModelBackend end

"""Select the scalarized OptimalControl modeling."""
struct OptimalControlScalarizedBackend <: AbstractModelBackend end

"""Select the ModelingToolkit modeling."""
struct ModelingToolkitBackend <: AbstractModelBackend end

# Keep the modeling table as the single place to add a backend.
const MODELING_SPECS = (
    (
        name=:OptimalControl,
        backend=OptimalControlBackend,
        directory="OptimalControlModels",
        weakdep=:OptimalControl,
    ),
    (
        name=:OptimalControlScalarized,
        backend=OptimalControlScalarizedBackend,
        directory="OptimalControlScalarizedModels",
        weakdep=:OptimalControl,
    ),
    (name=:JuMP, backend=JuMPBackend, directory="JuMPModels", weakdep=:JuMP),
    (
        name=:ModelingToolkit,
        backend=ModelingToolkitBackend,
        directory="ModelingToolkitModels",
        weakdep=:ModelingToolkit,
    ),
)

const MODELING_BY_TYPE = Dict(spec.backend => spec for spec in MODELING_SPECS)
const WEAKDEPS = Dict(spec.backend => spec.weakdep for spec in MODELING_SPECS)

# The `ext/` directory of the package, where the models of each modeling live.
const EXT_PATH = normpath(joinpath(@__DIR__, "..", "..", "ext"))

# The modeling directories are precompilation dependencies: adding a problem file must
# invalidate the package cache.
for spec in MODELING_SPECS
    Base.include_dependency(joinpath(EXT_PATH, spec.directory))
end

function _modeling_spec(::T) where {T<:AbstractModelBackend}
    return get(MODELING_BY_TYPE, T) do
        return throw(CTBase.Exceptions.IncorrectArgument("Unknown modeling backend $(T)"))
    end
end

function _problem_file(problem::Symbol, spec)
    return joinpath(EXT_PATH, spec.directory, "$(problem).jl")
end

function _problem_exists(problem::Symbol, spec)
    return isfile(_problem_file(problem, spec))
end
