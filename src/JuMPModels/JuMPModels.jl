module JuMPModels

using JuMP

path = dirname(@__FILE__)

files = filter(x -> x[(end - 2):end] == ".jl", readdir(path))
for file in files
    if file ≠ "JuMPModels.jl"
        include(file)
    end
end

end
