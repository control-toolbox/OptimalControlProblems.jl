module TestLint

using Test: Test
using OptimalControlProblems: OptimalControlProblems

const VERBOSE = isdefined(Main, :TestOptions) ? Main.TestOptions.VERBOSE : true
const SHOWTIMING = isdefined(Main, :TestOptions) ? Main.TestOptions.SHOWTIMING : true

include(joinpath(pkgdir(OptimalControlProblems), "tools", "lint", "ProblemLint.jl"))
const L = ProblemLint

# ------- an example problem, `foo`, that follows the template -------

const DESCRIPTION = """
The **foo problem**.

### Mathematical formulation

A formulation.

### Characteristics

- A characteristic.

### References

- **A. (2000)**. *Title*.
"""

const METADATA_TEXT = """
foo_meta = OrderedDict(
    :grid_size => 100,
    :parameters => (t0=0, tf=1, x_t0=0, x_u=1, guess=0.5),
    :parameter_info => (),
    :status => (OptimalControl=:valid,),
    :init => p -> (state=p.guess, control=0.0),
)
"""

const MODEL = """
\"\"\"The foo problem.\"\"\"
function OptimalControlProblems.foo(
    ::OptimalControlBackend; parameters::Union{Nothing,NamedTuple}=nothing, init=nothing
)
    OptimalControlProblems.Catalogue._warn_if_draft(
        :foo, :OptimalControl
    )
    params = parameters_data(:foo, parameters)
    t0, tf = params[:t0], params[:tf]
    x_t0, x_u = params[:x_t0], params[:x_u]
    ocp = @def begin
        t ∈ [t0, tf], time
        x ∈ R, state
        u ∈ R, control
        x(t0) == x_t0
        x(t) ≤ x_u
        ẋ(t) == u(t)
        ∫(u(t)^2) → min
    end
    init = OptimalControlProblems.Metadata._initial_guess(:foo, params, init)
    return OptimalControlProblems.Problems._problem(:foo, :OptimalControl, ocp, init, params)
end
"""

info(names...) = NamedTuple{names}(Tuple((unit=nothing, meaning="m") for _ in names))

function metadata(;
    status=(OptimalControl=:valid,), parameter_info=:default, grid_sizes=nothing
)
    parameters = (t0=0, tf=1, x_t0=0, x_u=1, guess=0.5)
    data = Dict{Symbol,Any}(:grid_size => 100, :parameters => parameters, :status => status)
    parameter_info === nothing || (
        data[:parameter_info] =
            parameter_info === :default ? info(keys(parameters)...) : parameter_info
    )
    isnothing(grid_sizes) || (data[:grid_sizes] = grid_sizes)
    return data
end

function example(; model=MODEL, jump=nothing, description=DESCRIPTION, kwargs...)
    return L.ProblemSource(
        :foo,
        metadata(; kwargs...),
        METADATA_TEXT,
        description,
        [
            L.Model(:OptimalControl, "OptimalControlBackend", model),
            L.Model(:JuMP, "JuMPBackend", jump),
        ],
    )
end

failures(src) = [r.message for r in L.checks(src) if r.status === :fail]
fails(src, needle) = any(m -> occursin(needle, m), failures(src))

function test_lint()
    Test.@testset "problem lint" verbose = VERBOSE showtiming = SHOWTIMING begin
        Test.@testset "the collection: no failure" begin
            results = L.lint()
            failed = filter(r -> r.status === :fail, results)
            isempty(failed) || print(L.report(results))
            Test.@test isempty(failed)
        end

        Test.@testset "an example that follows the template" begin
            Test.@test isempty(failures(example()))
        end

        Test.@testset "files and statuses" begin
            Test.@test fails(example(; description=nothing), "a description")
            # a status without a model file, a model file without a status
            Test.@test fails(
                example(; status=(OptimalControl=:valid, JuMP=:draft)), "no status for JuMP"
            )
            Test.@test fails(example(; status=NamedTuple()), "a status for OptimalControl")
        end

        Test.@testset "description" begin
            Test.@test fails(
                example(;
                    description=replace(DESCRIPTION, "### References" => "### Sources")
                ),
                "the outline",
            )
            no_entry = replace(DESCRIPTION, "- **A. (2000)**. *Title*." => "See the web.")
            Test.@test fails(
                example(; description=no_entry), "`### References` with entries"
            )
            # the outline: the order, the required headings, no parameters section
            Test.@test L.follows_outline(
                "### Mathematical formulation\n### Qualitative behaviour\n### Characteristics\n### References\n",
            )
            Test.@test !L.follows_outline("### Characteristics\n### References\n")
            Test.@test !L.follows_outline(
                "### Mathematical formulation\n### Parameters\n### Characteristics\n### References\n",
            )
            Test.@test !L.follows_outline(
                "### Mathematical formulation\n### References\n### Characteristics\n"
            )
            Test.@test L.cites_sources("text\n\n### References\n\n- **A. (2000)**. *T*.\n")
            Test.@test !L.cites_sources(
                "text\n\n### References\n\nSee below.\n\n### Other\n\n- an item\n"
            )
        end

        Test.@testset "parameter info" begin
            Test.@test fails(example(; parameter_info=nothing), "a `:parameter_info`")
            Test.@test fails(
                example(; parameter_info=info(:t0, :tf)), "describes exactly the parameters"
            )
            bad = merge(info(:t0, :tf, :x_t0, :x_u), (guess=(unit=nothing, meaning=""),))
            Test.@test fails(example(; parameter_info=bad), "not guess")
        end

        Test.@testset "grid per modeling" begin
            Test.@test !fails(example(; grid_sizes=(OptimalControl=50,)), "`:grid_sizes`")
            Test.@test fails(example(; grid_sizes=(JuMP=50,)), "`:grid_sizes`: a positive")
            Test.@test fails(
                example(; grid_sizes=(OptimalControl=0,)), "not OptimalControl"
            )
        end

        Test.@testset "template of a model" begin
            for (needle, what) in (
                ("\"\"\"The foo problem.\"\"\"\n" => "", "a docstring"),
                ("init=nothing" => "init", "the keyword `init=nothing`"),
                (":foo, :OptimalControl\n" => ":foo, :JuMP\n", "the draft warning"),
                (
                    "parameters_data(:foo, parameters)" => "parameters_data(:bar, parameters)",
                    "the parameters",
                ),
                ("_initial_guess(:foo," => "initial(:foo,", "the initial guess"),
                ("_problem(:foo, :OptimalControl," => "_problem(:foo, :JuMP,", "_problem"),
            )
                Test.@test fails(example(; model=replace(MODEL, needle)), what)
            end
            # JuMP: its own keywords
            jump = replace(
                MODEL,
                "::OptimalControlBackend; parameters" => "::JuMPBackend, args...; parameters",
                ":foo, :OptimalControl\n" => ":foo, :JuMP\n",
            )
            src = example(; jump, status=(OptimalControl=:valid, JuMP=:draft))
            Test.@test fails(src, "the keyword `grid_size`")
            Test.@test fails(src, "the keyword `scheme`")
        end

        Test.@testset "parameters of a model" begin
            # read and not used
            unused = replace(MODEL, "x(t) ≤ x_u" => "x(t) ≤ 1 + 0")
            Test.@test fails(
                example(; model=unused), "uses every parameter it reads: not x_u"
            )
            # read but not a parameter of the problem
            unknown = replace(MODEL, "params[:x_u]" => "params[:x_max]")
            Test.@test fails(example(; model=unknown), "not x_max")
            # a parameter read by nothing: neither a model nor the metadata
            Test.@test fails(
                example(;
                    model=replace(
                        MODEL,
                        "x_t0, x_u = params[:x_t0], params[:x_u]" => "x_t0, x_u = params[:x_t0], 1.0",
                        "x(t) ≤ x_u" => "x(t) ≤ x_u",
                    ),
                ),
                "read by a model or by the metadata",
            )
            # `guess` is read by the initial guess of the metadata only: fine
            Test.@test !fails(example(), "not guess")
            Test.@test L.parameters_read("(; a, b) = params\nc = params[:c]") ==
                ["c", "a", "b"]
        end

        Test.@testset "bounds written in the code" begin
            Test.@test fails(
                example(; model=replace(MODEL, "x(t) ≤ x_u" => "x(t) ≤ 1.5")),
                "no bound written in the code",
            )
            Test.@test L.literal_bounds("    -1.57 ≤ u(t) ≤ 1.57") ==
                ["-1.57 ≤ u(t) ≤ 1.57"]
            Test.@test L.literal_bounds("    tf ≥ 0") == ["tf ≥ 0"]
            Test.@test L.literal_bounds("Component(\"v\"; lower=1.0)") ==
                ["Component(\"v\"; lower=1.0)"]
            # an index, a subscript, an exponent or a comment is not a bound
            Test.@test isempty(L.literal_bounds("    x[1](t) ≥ x₁_l"))
            Test.@test isempty(L.literal_bounds("    x₁(t) ≤ x1_max"))
            Test.@test isempty(L.literal_bounds("    u(t)^2 ≤ u_u # at most 2"))
        end

        Test.@testset "report" begin
            results = L.checks(example(; model=replace(MODEL, "x(t) ≤ x_u" => "x(t) ≤ 2")))
            text = L.report(results)
            Test.@test occursin("❌ 2 failure(s)", text)
            Test.@test occursin("| ❌ | `foo` | model OptimalControl |", text)
            # it always says what it does not check
            Test.@test occursin("Checked by other tools", text)
            Test.@test occursin("To check by hand, for `foo`", text)
            Test.@test occursin("- [ ] the mathematical formulation", text)
            Test.@test occursin("not that a problem is right", text)
            Test.@test !occursin("| ✅ | `foo`", text)
            Test.@test occursin("| ✅ | `foo`", L.report(results; verbose=true))
            Test.@test occursin("✅ no failure", L.report(L.checks(example())))
        end
    end
end

end # module TestLint

test_lint() = TestLint.test_lint()
