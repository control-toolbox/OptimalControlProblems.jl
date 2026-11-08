# The problems of the collection: one metadata file per problem, `ext/MetaData/<problem>.jl`.
function make_list_of_problems()
    path = joinpath(@__DIR__, "..", "..", "ext", "MetaData")
    Base.include_dependency(path)
    files = filter(endswith(".jl"), readdir(path))
    return tuple(sort!(Symbol.(first.(splitext.(files))))...), path
end

const PROBLEM_NAMES, METADATA_PATH = make_list_of_problems()
