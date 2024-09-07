module OptimalControlProblems

using CTBase

abstract type AbstractModelBackend end
struct JuMPBackend <: AbstractModelBackend end
struct OptimalControlBackend <: AbstractModelBackend end

# weak dependencies
weakdeps = Dict(OptimalControlBackend => :CTDirect, JuMPBackend => :JuMP)

# path to problems
path = joinpath(dirname(@__FILE__), "..", "ext", "MetaData")

# ------- Problem Definitions -------
files = filter(x -> x[(end - 2):end] == ".jl", readdir(path))
for file in files
    problem = Symbol(file[1:(end - 3)])
    code = quote
        function $problem(model_backend::T, args...; kwargs...) where {T<:AbstractModelBackend}
            throw(ExtensionError(weakdeps[T]))
        end
        export $problem
    end
    eval(code)
end

export JuMPBackend, OptimalControlBackend

end
