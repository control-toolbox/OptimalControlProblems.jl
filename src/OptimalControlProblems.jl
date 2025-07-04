module OptimalControlProblems

using CTBase

abstract type AbstractModelBackend end
struct JuMPBackend <: AbstractModelBackend end
struct OptimalControlBackend <: AbstractModelBackend end

# weak dependencies
weakdeps = Dict(OptimalControlBackend => :OptimalControl, JuMPBackend => :JuMP)

# path to problems
path = joinpath(dirname(@__FILE__), "..", "ext", "MetaData")

# ------- Problem Definitions -------
files = filter(x -> x[(end - 2):end] == ".jl", readdir(path))
for file in files
    problem = Symbol(file[1:(end - 3)])
    code = quote
        function $problem(model_backend::T, args...; kwargs...) where {T<:AbstractModelBackend}
            throw(CTBase.ExtensionError(weakdeps[T]))
        end
        export $problem
    end
    eval(code)
end

# ------- Problem Metadata -------
for file in files
    include(joinpath(path, file))
end
number_of_problems = length(files)

const infos = [
    :name
    :nh
    :nvar
    :ncon
    :minimize
    :state_name
    :costate_name
    :control_name
    :time
]

const types = [
    Union{String,Nothing},
    Union{Int,Nothing},
    Union{Int,Nothing},
    Union{Int,Nothing},
    Union{Bool,Nothing},
    Union{Vector{String}, String, Nothing}, 
    Union{Vector{String}, String, Nothing}, 
    Union{Vector{String}, String, Nothing},  
    Union{Tuple{String, String, Union{Real,Nothing}}, Nothing},
]

"""
OptimalControlProblems.metadata
---
The following keys are valid:
    - `name::String`: problem name
    - `nh::Int`: default number of discretization points
    - `nvar::Int`: number of variables
    - `ncon::Int`: number of general constraints
    - `minimize::Bool`: true if optimize == minimize
"""
const metadata = Dict()

for i in 1:number_of_problems
    file_key = Symbol(split(files[i], ".")[1])
    metadata[file_key] = Dict()
    for (data, T) in zip(infos, types)
        value = eval(Meta.parse("$(file_key)_meta"))[data]
        if !(value isa T)
            error("Type mismatch: Expected $(T) for $(data), but got $(typeof(value))")
        end
        metadata[file_key][data] = value
    end
end

# ------- Available Problems Function -------
"""
    available_problems()

Returns the list of problems that are currently working based on the last test execution.
The list is read from a cache file that gets updated every time tests are run.
If no cache exists, returns an empty list with a warning to run tests first.
"""
function available_problems()
    cache_file = joinpath(dirname(@__FILE__), "..", "available_problems_cache.txt")
    if isfile(cache_file)
        try
            content = read(cache_file, String)
            if !isempty(strip(content))
                # Parse symbols from the file
                lines = split(strip(content), '\n')
                return [Symbol(strip(line)) for line in lines if !isempty(strip(line))]
            end
        catch e
            @warn "Error reading the cache: $e"
        end
    end
    # Default list if the cache does not exist or is empty
    @warn "Available problems cache not found. Run the tests to update the list."
    return Symbol[]
end

export JuMPBackend, OptimalControlBackend, available_problems

end
