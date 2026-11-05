# The runtime capabilities of the machine the tests run on, defined once (TST6): a test
# reads `Main.TestCapabilities.*` and never asks CUDA itself. Included by the files of the
# environment `GPU`, which has CUDA; the CPU environments do not include it.
module TestCapabilities

using CUDA: CUDA

"""A functional CUDA device is available."""
const CUDA_FUNCTIONAL = CUDA.functional()

"""
The tests run on a self-hosted GPU runner (`kkt`, `occidata`): the GPU tests are then
required, not skipped. `RUNNER_NAME` is set by the GitHub Actions agent to the registered
name of the runner (`kkt-runner`, `occidata-runner`).
"""
const ON_GPU_RUNNER = any(
    gpu -> occursin(gpu, get(ENV, "RUNNER_NAME", "")), ("kkt", "occidata")
)

end # module TestCapabilities
