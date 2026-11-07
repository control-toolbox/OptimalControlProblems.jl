# Precompiles the solve of a few problems with ADNLP and Ipopt (#373, prototype).
module PrecompileIpopt

using OptimalControlProblems: OptimalControlProblems
using OptimalControl: OptimalControl
using NLPModelsIpopt: NLPModelsIpopt
using NLPModels: NLPModels # with the solver, loads its strategy in CTSolvers
using PrecompileTools: PrecompileTools

include(joinpath("Precompile", "workload.jl"))

PrecompileTools.@compile_workload begin
    workload(
        OptimalControl.ADNLP(), OptimalControl.Ipopt(; print_level=0, sb="yes", max_iter=1)
    )
end

end
