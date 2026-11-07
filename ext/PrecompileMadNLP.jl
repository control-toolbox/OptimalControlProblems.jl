# Precompiles the solve of a few problems with Exa and MadNLP (#373, prototype).
module PrecompileMadNLP

using OptimalControlProblems: OptimalControlProblems
using OptimalControl: OptimalControl
using MadNLP: MadNLP
using NLPModels: NLPModels # with the solver, loads its strategy in CTSolvers
using PrecompileTools: PrecompileTools

include(joinpath("Precompile", "workload.jl"))

PrecompileTools.@compile_workload begin
    workload(
        OptimalControl.Exa(), OptimalControl.MadNLP(; print_level=MadNLP.ERROR, max_iter=1)
    )
end

end
