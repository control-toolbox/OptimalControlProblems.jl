module OptimalControlProblems

greet() = print("Hello World!")

include("OptimalControlModels/OptimalControlModels.jl")
include("JuMPModels/JuMPModels.jl")

export JuMPModels, OptimalControlModels

end
