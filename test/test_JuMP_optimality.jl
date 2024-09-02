using Ipopt

# test_JuMP_optimality
function test_JuMP_optimality()
    model = OptimalControlProblems.JuMPModels.cart_pendulum()
    # Solve the model
    optimize!(model)
    # Test that the solver found an optimal solution
    @test termination_status(model) == MOI.OPTIMAL
end
