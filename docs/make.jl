# To generate the documentation: `julia --project=docs docs/make.jl`
# To serve it: `npx serve docs/build/1 --listen 5173`
# Or, with Julia only:
# `julia --project=docs -e 'using LiveServer; LiveServer.serve(dir="docs/build/1", single_page=true)'`

# Append (not prepend): docs/Manifest.toml must win for shared deps; the repo root is only
# needed to find the local package (SITE18).
push!(LOAD_PATH, joinpath(@__DIR__))
push!(LOAD_PATH, joinpath(@__DIR__, ".."))

using Documenter           # Core package to generate documentation
using DocumenterVitepress  # VitePress backend and deployment
using CTBase               # automatic API reference
using OptimalControlProblems
using OptimalControl       # loads the OptimalControl extension
using JuMP                 # loads the JuMP extension
using Plots

# The extensions, by name, for the `@docs` blocks of the API Reference.
const JuMPModels = Base.get_extension(OptimalControlProblems, :JuMPModels)
const OptimalControlModels = Base.get_extension(
    OptimalControlProblems, :OptimalControlModels
)

# Plots are saved as SVG (SITE12): DocumenterVitepress prefers PNG when both are offered.
Base.showable(::MIME"image/png", ::Plots.Plot) = false

include("modelings.jl")    # documentation adapters, one per modeling
include("problems.jl")     # the problem pages
include("results.jl")      # the results of the crons, drawn on the problem pages
include("browser.jl")      # the interactive problems browser
include("status.jl")       # the status page
include("api_reference.jl")

# ==============================
# --- Copy Project Assets for reproducibility ---
# ==============================
mkpath(joinpath(@__DIR__, "src", "assets"))  # Ensure assets folder exists
mkpath(joinpath(@__DIR__, "src", "public", "assets"))

for file in ("Manifest.toml", "Project.toml")
    cp(joinpath(@__DIR__, file), joinpath(@__DIR__, "src", "assets", file); force=true)
    cp(
        joinpath(@__DIR__, file),
        joinpath(@__DIR__, "src", "public", "assets", file);
        force=true,
    )
end

# Repository URL (used for links in docs)
repo_url = "github.com/control-toolbox/OptimalControlProblems.jl"

# ==============================
# --- Generated pages ---
# ==============================
# The problem pages execute no code: they are written from the files of each problem, and
# show the results the crons published (#314).
load_results!()
PROBLEM_PAGES = generate_problem_pages()

# If draft is true, the Julia code of the hand-written pages is not executed. To disable
# draft mode in a specific markdown file, add:
#=
```@meta
Draft = false
```
=#
draft = false

src_dir = abspath(joinpath(@__DIR__, "..", "src"))
ext_dir = abspath(joinpath(@__DIR__, "..", "ext"))

# Where the site is deployed, decided once (SITE13). DocumenterVitepress builds the site for
# the base of this decision; without it, it decides with `push_preview=true` and builds a pull
# request for `previews/PR<n>`, announcing a deployment that `deploydocs` then refuses. A pull
# request deploys its preview only with the label `documentation push preview`, which sets
# `DOCUMENTER_PUSH_PREVIEW` (CTActions).
push_preview = get(ENV, "DOCUMENTER_PUSH_PREVIEW", "false") == "true"
deploy_decision = Documenter.deploy_folder(
    Documenter.auto_detect_deploy_system();
    repo=repo_url,
    devbranch="main",
    devurl="dev",
    push_preview=push_preview,
)

with_status_page() do status_file
    with_problems_browser() do browser_file
        with_api_reference(src_dir, ext_dir) do api_pages
            makedocs(;
                draft=draft,
                warnonly=:cross_references,
                sitename="OptimalControlProblems.jl",
                format=DocumenterVitepress.MarkdownVitepress(;
                    repo=repo_url,
                    devbranch="main",
                    devurl="dev",
                    sidebar_drawer=true,
                    deploy_decision=deploy_decision,
                ),
                pages=[
                    "Introduction" => "index.md",
                    "Getting started" => [
                        "Installation" => "getting-started/installation.md",
                        "Your first problem" => "getting-started/first-problem.md",
                        "Guided tour" => "getting-started/guided-tour.md",
                    ],
                    "User guide" => [
                        "Get a problem" => "guide/get.md",
                        "Solve with OptimalControl" => "guide/solve-optimalcontrol.md",
                        "Solve with JuMP" => "guide/solve-jump.md",
                        "JuMP schemes" => "guide/jump-schemes.md",
                        "Initial guess" => "guide/initial-guess.md",
                        "Compare modelings" => "guide/compare.md",
                        "Plot a solution" => "guide/plot.md",
                    ],
                    "Problems" => [
                        "Overview" => "problems-overview.md",
                        "Browser" => browser_file,
                        "Status" => status_file,
                        "All the problems" => PROBLEM_PAGES,
                    ],
                    "Developers" => "developers.md",
                    "API Reference" => api_pages,
                    "Migrating from 0.5" => "migration.md",
                ],
            )
        end
    end
end

# ==============================
# --- Deploy Documentation ---
# ==============================
bases_file = joinpath(@__DIR__, "build", "bases.txt")
if !deploy_decision.all_ok
    reason = if get(ENV, "GITHUB_EVENT_NAME", "") == "pull_request" && !push_preview
        "no preview requested (label `documentation push preview` absent)"
    else
        "the deployment criteria above are not met"
    end
    @info "Skipping deployment: $(reason)."
elseif !isfile(bases_file)
    @info "Skipping deployment: no bases were built."
else
    DocumenterVitepress.deploydocs(;
        repo=repo_url * ".git", devbranch="main", push_preview=push_preview
    )
end
