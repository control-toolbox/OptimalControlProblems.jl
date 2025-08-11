#using CTBase
using Ipopt
using JuMP
using NLPModelsIpopt
using OptimalControl
using OptimalControlProblems
using Test
include("utils.jl") 

# Parameters for the solvers
const TOL = 1e-8
const MU_STRATEGY = "adaptive"
const SB = "yes"
const constr_viol_tol = 1e-8
const MAX_ITER = 1000
const MAX_WALL_TIME = 500.0

list_of_problems=[:electric_vehicle]
verbose = true

include("test_JuMP.jl")
test_JuMP()

include("test_OptimalControl.jl")
test_OptimalControl()