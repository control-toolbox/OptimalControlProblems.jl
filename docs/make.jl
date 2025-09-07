# ==============================
# --- Import Packages & Modules ---
# ==============================
using Documenter           # Core package to generate documentation
using DocumenterInterLinks # For linking to other package docs
using DocumenterMermaid    # For Mermaid diagrams in docs
using OptimalControlProblems
using OptimalControl
using JuMP
using CTModels
using ExaModels
include("problems.jl")    # Your problem definitions
include("browser.jl")     # Your interactive problems browser

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
        "https://control-toolbox.org/OptimalControl.jl/stable/",
        "https://control-toolbox.org/OptimalControl.jl/stable/objects.inv",
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

cp(
    joinpath(@__DIR__, "Manifest.toml"),
    joinpath(@__DIR__, "src", "assets", "Manifest.toml");
    force=true,
)
cp(
    joinpath(@__DIR__, "Project.toml"),
    joinpath(@__DIR__, "src", "assets", "Project.toml");
    force=true,
)

# Repository URL (used for links in docs)
repo_url = "github.com/control-toolbox/OptimalControlProblems.jl"

# ==============================
# --- Generate Problems Documentation ---
# ==============================
draft = false  # If true, code blocks in markdown are not executed
exclude_from_draft=Symbol[
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
with_problems_browser() do path # generates the problems browser and remove it at the end

    makedocs(;
        draft=draft,
        #remotes=nothing,
        warnonly=:cross_references,
        sitename="OptimalControlProblems.jl",
        format=Documenter.HTML(;
            repolink="https://" * repo_url,
            prettyurls=false,
            size_threshold_ignore=["dev-api.md", PROBLEMS_PAGES...],
            assets=[
                asset("https://control-toolbox.org/assets/css/documentation.css"),
                asset("https://control-toolbox.org/assets/js/documentation.js"),
            ],
        ),
        pages=[
            "Getting Started" => "index.md",
            "Problems" =>
                ["problems-introduction.md", "problems_browser.md", "List of the problems" => PROBLEMS_PAGES],
            "Tutorials" => [
                "Get a problem" => "tutorial-get.md",
                "Solve a problem" => "tutorial-solve.md",
            ],
            "Developers" => ["Add a problem" => "dev-add.md", "API" => "dev-api.md"],
        ],
        plugins=[links],
    )

end

# ==============================
# --- Deploy Documentation ---
# ==============================
deploydocs(; repo=repo_url * ".git", devbranch="main", push_preview=true)
