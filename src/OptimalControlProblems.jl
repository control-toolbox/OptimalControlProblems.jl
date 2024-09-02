module OptimalControlProblems

include("./OptimalControlModels/OptimalControlModels.jl")
include("./JuMPModels/JuMPModels.jl")

export JuMPModels, OptimalControlModels

greet() = print("Hello World!")

end
