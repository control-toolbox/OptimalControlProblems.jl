function test_modelings()
    @testset "modeling lists" begin
        @test issorted(problems())
        @test :moonlander in problems(JuMPBackend())
        @test :moonlander in problems(OptimalControlBackend())
        @test :moonlander ∉ problems(OptimalControlScalarizedBackend())
        @test :moonlander in problems(; status=:all)
        @test :moonlander ∉ problems(OptimalControlScalarizedBackend(); status=:all)
        @test problems(JuMPBackend(), OptimalControlBackend()) ==
              intersect(problems(JuMPBackend()), problems(OptimalControlBackend()))
        @test problems(OptimalControlBackend(), OptimalControlScalarizedBackend()) ==
              intersect(
                  problems(OptimalControlBackend()),
                  problems(OptimalControlScalarizedBackend()),
              )
    end

    @testset "missing modeling" begin
        error = try
            moonlander(OptimalControlScalarizedBackend())
            nothing
        catch exception
            exception
        end
        @test error isa CTBase.IncorrectArgument
        @test occursin("problems(OptimalControlScalarizedBackend())", sprint(showerror, error))
    end

    @testset "draft warning" begin
        data = metadata(:beam)
        original_status = data[:status]
        data[:status] = merge(original_status, (JuMP=:draft,))
        try
            @test_logs (:warn, r"beam is a draft for JuMP") match_mode=:any begin
                beam(JuMPBackend(); grid_size=2)
            end
        finally
            data[:status] = original_status
        end
    end

    @testset "generated documentation" begin
        documentation = string(Base.Docs.doc(beam))
        @test !occursin("TYPEDSIGNATURES", documentation)
        @test !occursin("TypedMethodSignatures", documentation)
        @test occursin("clamped beam problem", lowercase(documentation))
        @test occursin("grid_size", documentation)
    end
end
