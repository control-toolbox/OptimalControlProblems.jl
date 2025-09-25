function test_parameters()

    @testset "merge (parameters)" verbose=VERBOSE begin
        @test OptimalControlProblems.merge(nothing, nothing) === nothing
        @test OptimalControlProblems.merge((tf = 1,), nothing ) == (tf = 1,)
        @test OptimalControlProblems.merge((tf = 1,), (tf = 2,)) == (tf = 2,)
        @test OptimalControlProblems.merge((tf = 1, a = 2), (tf = 2,)) == (tf = 2, a = 2)
        @test_throws CTBase.UnauthorizedCall OptimalControlProblems.merge(nothing, (tf = 1,))
    end

end