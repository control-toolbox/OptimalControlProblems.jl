# Consistency test of a modeling environment (#304): after loading the extension, every
# problem having a model file has a real method of the extension, not only the generic
# fallback of OptimalControlProblems.
module TestMethods

using Test: Test
using OptimalControlProblems: OptimalControlProblems

"""Check the methods of `backend`, defined by the extension `extension` (a `Symbol`)."""
function test_methods(backend, extension::Symbol)
    ext = Base.get_extension(OptimalControlProblems, extension)
    Test.@test !isnothing(ext) # the extension is loaded in this environment
    isnothing(ext) && return nothing
    Main.TestStatus.for_each_problem("methods", backend) do problem
        method = which(getfield(OptimalControlProblems, problem), (typeof(backend),))
        Test.@test parentmodule(method) === ext
    end
    return nothing
end

end # module TestMethods
