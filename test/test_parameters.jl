function test_parameters()

    @testset "merge (parameters)" verbose=VERBOSE begin
        @test OptimalControlProblems.merge(nothing, nothing) === nothing
        @test OptimalControlProblems.merge((tf = 1,), nothing ) == (tf = 1,)
        @test OptimalControlProblems.merge((tf = 1,), (tf = 2,)) == (tf = 2,)
        @test OptimalControlProblems.merge((tf = 1, a = 2), (tf = 2,)) == (tf = 2, a = 2)
        @test_throws CTBase.UnauthorizedCall OptimalControlProblems.merge(nothing, (tf = 1,))
    end

    for problem in LIST_OF_PROBLEMS
        @testset "$(string(problem)) (parameters)" verbose=VERBOSE begin
            t0 = 1000
            tf = 2000

            # get info from the problem
            docp = OptimalControlProblems.eval(problem)(OptimalControlBackend())
            ocp = ocp_model(docp)
            t0_fixed = has_fixed_initial_time(ocp)
            tf_fixed = has_fixed_final_time(ocp)
            t0_name = metadata(problem)[:time_grid_names][:initial_time]
            tf_name = metadata(problem)[:time_grid_names][:final_time]

            if t0_fixed
                params = NamedTuple(Dict(Symbol(t0_name) => t0, ))

                docp = OptimalControlProblems.eval(problem)(OptimalControlBackend(); parameters=params)
                @test initial_time(ocp_model(docp)) == t0

                docp = OptimalControlProblems.eval(Symbol(problem, :_s))(OptimalControlBackend(); parameters=params)
                @test initial_time(ocp_model(docp)) == t0

                nlp = OptimalControlProblems.eval(problem)(JuMPBackend(); parameters=params)
                @test nlp[Symbol(t0_name)] == t0
            end

            if tf_fixed
                params = NamedTuple(Dict(Symbol(tf_name) => tf, ))

                docp = OptimalControlProblems.eval(problem)(OptimalControlBackend(); parameters=params)
                @test final_time(ocp_model(docp)) == tf

                docp = OptimalControlProblems.eval(Symbol(problem, :_s))(OptimalControlBackend(); parameters=params)
                @test final_time(ocp_model(docp)) == tf

                nlp = OptimalControlProblems.eval(problem)(JuMPBackend(); parameters=params)
                @test nlp[Symbol(tf_name)] == tf
            end

            if t0_fixed && tf_fixed
                params = NamedTuple(Dict(Symbol(t0_name) => t0, Symbol(tf_name) => tf))

                docp = OptimalControlProblems.eval(problem)(OptimalControlBackend(); parameters=params)
                @test initial_time(ocp_model(docp)) == t0
                @test final_time(ocp_model(docp)) == tf

                docp = OptimalControlProblems.eval(Symbol(problem, :_s))(OptimalControlBackend(); parameters=params)
                @test initial_time(ocp_model(docp)) == t0
                @test final_time(ocp_model(docp)) == tf

                nlp = OptimalControlProblems.eval(problem)(JuMPBackend(); parameters=params)
                @test nlp[Symbol(t0_name)] == t0
                @test nlp[Symbol(tf_name)] == tf
            end

        end
    end


end