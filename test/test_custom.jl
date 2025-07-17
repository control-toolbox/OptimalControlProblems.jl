using Ipopt
using JuMP
using NLPModelsIpopt
using OptimalControlProblems
using Test
include("utils.jl") 

# Parameters for the solvers
const tol = 1e-8
const mu_strategy = "adaptive"
const sb = "yes"
const constr_viol_tol = 1e-8
const max_iter = 1000
const max_wall_time = 500.0

list_of_problems=[:cart_pendulum]
verbose = true

include("test_JuMP.jl")
test_JuMP()