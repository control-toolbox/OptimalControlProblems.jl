module OptimalControlModels

using OptimalControl

path = dirname(@__FILE__)

files = filter(x -> x[(end - 2):end] == ".jl", readdir(path))
for file in files
    if file ≠ "OptimalControlModels.jl"
        include(file)
    end
end

end