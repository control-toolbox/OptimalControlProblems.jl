using Ipopt
using JuMP 

# test_JuMP_optimality
function test_JuMP_optimality()
    # Set up the model
    model = OptimalControlProblems.JuMPModels.cart_pendulum()
    set_optimizer(model,Ipopt.Optimizer)
    set_silent(model)
    set_optimizer_attribute(model,"tol",1e-8)
    set_optimizer_attribute(model,"constr_viol_tol",1e-6)
    set_optimizer_attribute(model,"max_iter",1000)
    set_optimizer_attribute(model,"mu_strategy","adaptive")
    set_optimizer_attribute(model,"linear_solver","mumps")
    # Solve the model
    Ipopt.optimize!(model)
    # Test that the solver found an optimal solution
    @test termination_status(model) == MOI.OPTIMAL
end
