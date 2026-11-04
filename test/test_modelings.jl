# No problem name is written here: everything is derived from the catalogue (the metadata
# and the model files are the single source of truth).
function test_modelings()
    backends = (JuMPBackend(), OptimalControlBackend(), OptimalControlScalarizedBackend())
    all_problems = problems(; status=:all)

    @testset "modeling lists" begin
        @test issorted(all_problems)
        @test !isempty(problems())
        for backend in backends
            listed = problems(backend; status=:all)
            @test listed ⊆ all_problems
            @test sort!(
                vcat(problems(backend; status=:valid), problems(backend; status=:draft))
            ) == listed
        end
        for a in backends, b in backends
            @test problems(a, b) == intersect(problems(a), problems(b))
        end
        @test_throws CTBase.IncorrectArgument problems(; status=:unknown)
    end

    @testset "status" begin
        for problem in all_problems
            statuses = OptimalControlProblems.status(problem)
            for backend in backends
                name = OptimalControlProblems._modeling_spec(backend).name
                # a status is declared exactly for the modelings having a model file
                @test haskey(statuses, name) == (problem in problems(backend; status=:all))
            end
        end
        @test_throws CTBase.IncorrectArgument OptimalControlProblems.status(:not_a_problem)
    end

    @testset "missing modeling" begin
        for backend in backends
            absent = setdiff(all_problems, problems(backend; status=:all))
            isempty(absent) && continue
            build = getfield(OptimalControlProblems, first(absent))
            err = try
                build(backend)
                nothing
            catch exception
                exception
            end
            @test err isa CTBase.IncorrectArgument
            @test occursin("problems(", sprint(showerror, err))
        end
    end

    @testset "draft warning" begin
        problem = first(problems(JuMPBackend()))
        data = metadata(problem)
        original_status = data[:status]
        @test_logs getfield(OptimalControlProblems, problem)(JuMPBackend(); grid_size=2)
        data[:status] = merge(original_status, (JuMP=:draft,))
        try
            @test_logs (:warn, Regex("$problem is a draft for JuMP")) match_mode = :any getfield(
                OptimalControlProblems, problem
            )(
                JuMPBackend(); grid_size=2
            )
        finally
            data[:status] = original_status
        end
    end

    @testset "generated documentation" begin
        for problem in all_problems
            documentation = string(Base.Docs.doc(getfield(OptimalControlProblems, problem)))
            @test !occursin("TYPEDSIGNATURES", documentation)
            @test !occursin("TypedMethodSignatures", documentation)
        end
    end
end
