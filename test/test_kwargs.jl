function test_kwargs()
    N = 2
    scheme = :euler
    solver_backend = :madnlp
    optimiser = Ipopt.Optimizer

    for f in LIST_OF_PROBLEMS
        @testset "$(string(f)) (kwargs)" verbose=VERBOSE begin
            DEBUG && println("─ ", string(f), " (kwargs)")

            # OptimalControl model
            docp = OptimalControlProblems.eval(f)(
                OptimalControlBackend(), solver_backend; grid_size=grid_size, disc_method=scheme
            )
            @test docp isa CTDirect.DOCP
            @test docp.time.steps == N
            @test docp.discretization isa CTDirect.Euler

            # OptimalControl_s model
            docp = OptimalControlProblems.eval(Symbol(f, :_s))(
                OptimalControlBackend(), :madnlp, :exa; grid_size=grid_size, disc_method=scheme
            )
            @test docp isa CTDirect.DOCP
            @test docp.time.steps == N
            @test docp.discretization isa CTDirect.Euler

            # JuMP model
            nlp = OptimalControlProblems.eval(f)(JuMPBackend(), optimiser; add_bridges=true)
            @test solver_name(nlp) == "Ipopt"
        end
    end
end
