# To generate the documentation: `julia --project=docs docs/make.jl`
# To serve it: `npx serve docs/build/1 --listen 5173`
# Or, with Julia only:
# `julia --project=docs -e 'using LiveServer; LiveServer.serve(dir="docs/build/1", single_page=true)'`

push!(LOAD_PATH, joinpath(@__DIR__))
push!(LOAD_PATH, joinpath(@__DIR__, ".."))

using Documenter           # Core package to generate documentation
using DocumenterInterLinks # For linking to other package docs
using DocumenterMermaid    # For Mermaid diagrams in docs
using DocumenterVitepress  # VitePress backend and deployment
using OptimalControlProblems
using OptimalControl
using JuMP
using CTModels
include("problems.jl")    # Your problem definitions

# ==============================
# --- Add Docstrings from external packages ---
# ==============================
# Access extensions within OptimalControlProblems
const JuMPModels = Base.get_extension(OptimalControlProblems, :JuMPModels)
const OptimalControlModels = Base.get_extension(
    OptimalControlProblems, :OptimalControlModels
)

# Add DocTest setup for each module if missing
Modules = [JuMPModels, OptimalControlModels]
for Module in Modules
    isnothing(DocMeta.getdocmeta(Module, :DocTestSetup)) &&
        DocMeta.setdocmeta!(Module, :DocTestSetup, :(using $Module); recursive=true)
end

# ==============================
# --- Configure inter-package links ---
# ==============================
links = InterLinks(
    "ADNLPModels" => (
        "https://jso.dev/ADNLPModels.jl/stable/",
        "https://jso.dev/ADNLPModels.jl/stable/objects.inv",
        joinpath(@__DIR__, "inventories", "ADNLPModels.toml"),
    ),
    "CTDirect" => (
        "https://control-toolbox.org/CTDirect.jl/stable/",
        "https://control-toolbox.org/CTDirect.jl/stable/objects.inv",
        joinpath(@__DIR__, "inventories", "CTDirect.toml"),
    ),
    "JuMP" => (
        "https://jump.dev/JuMP.jl/stable/",
        "https://jump.dev/JuMP.jl/stable/objects.inv",
        joinpath(@__DIR__, "inventories", "JuMP.toml"),
    ),
    "NLPModelsIpopt" => (
        "https://jso.dev/NLPModelsIpopt.jl/stable/",
        "https://jso.dev/NLPModelsIpopt.jl/stable/objects.inv",
        joinpath(@__DIR__, "inventories", "NLPModelsIpopt.toml"),
    ),
    "NLPModelsJuMP" => (
        "https://jso.dev/NLPModelsJuMP.jl/stable/",
        "https://jso.dev/NLPModelsJuMP.jl/stable/objects.inv",
        joinpath(@__DIR__, "inventories", "NLPModelsJuMP.toml"),
    ),
    "OptimalControl" => (
        "https://control-toolbox.org/OptimalControl.jl/dev/",
        "https://control-toolbox.org/OptimalControl.jl/dev/objects.inv",
        joinpath(@__DIR__, "inventories", "OptimalControl.toml"),
    ),
    "Tutorials" => (
        "https://control-toolbox.org/Tutorials.jl/stable/",
        "https://control-toolbox.org/Tutorials.jl/stable/objects.inv",
        joinpath(@__DIR__, "inventories", "Tutorials.toml"),
    ),
)

# ==============================
# --- Copy Project Assets for reproducibility ---
# ==============================
mkpath(joinpath(@__DIR__, "src", "assets"))  # Ensure assets folder exists
mkpath(joinpath(@__DIR__, "src", "public", "assets"))

for file in ("Manifest.toml", "Project.toml")
    cp(
        joinpath(@__DIR__, file),
        joinpath(@__DIR__, "src", "assets", file);
        force=true,
    )
    cp(
        joinpath(@__DIR__, file),
        joinpath(@__DIR__, "src", "public", "assets", file);
        force=true,
    )
end

# Repository URL (used for links in docs)
repo_url = "github.com/control-toolbox/OptimalControlProblems.jl"

# ==============================
# --- Generate Problems Documentation ---
# ==============================
draft = false  # If true, code blocks in markdown are not executed
exclude_from_draft = Symbol[
#    :beam   # example: exclude beam from draft docs
]

# Generate problem pages (returns list of markdown pages)
PROBLEMS_PAGES = generate_documentation_problems(;
    draft=draft, exclude_from_draft=exclude_from_draft
)

# ==============================
# --- Generate Documentation ---
# ==============================
# If draft is true, the Julia code in markdown is not executed.
# To disable draft mode in a specific markdown file, add:
#=
```@meta
Draft = false
```
=#
makedocs(;
    draft=draft,
    #remotes=nothing,
    warnonly=:cross_references,
    sitename="OptimalControlProblems.jl",
    format=DocumenterVitepress.MarkdownVitepress(;
        repo=repo_url,
        devbranch="main",
        devurl="dev",
        sidebar_drawer=true,
    ),
    pages=[
        "Problems" => [
            "problems-introduction.md",
            "List of the problems" => PROBLEMS_PAGES,
        ],
        "Tutorials" => [
            "Get a problem" => "tutorial-get.md",
            "Solve a problem" => "tutorial-solve.md",
        ],
        "Developers" => ["Add a problem" => "dev-add.md", "API" => "dev-api.md"],
    ],
    plugins=[links],
)

# ==============================
# --- Deploy Documentation ---
# ==============================
bases_file = joinpath(@__DIR__, "build", "bases.txt")
if isfile(bases_file)
    DocumenterVitepress.deploydocs(;
        repo=repo_url * ".git",
        devbranch="main",
        push_preview=get(ENV, "DOCUMENTER_PUSH_PREVIEW", "false") == "true",
    )
else
    @info "Skipping deployment: no bases were built."
end
