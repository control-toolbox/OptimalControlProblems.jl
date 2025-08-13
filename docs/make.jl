using Documenter
using DocumenterInterLinks

#
links = InterLinks(
    "CTDirect" => (
        "https://control-toolbox.org/CTDirect.jl/stable/",
        "https://control-toolbox.org/CTDirect.jl/stable/objects.inv",
        joinpath(@__DIR__, "inventories", "CTDirect.toml"),
    ),
    "ADNLPModels" => (
        "https://jso.dev/ADNLPModels.jl/stable/",
        "https://jso.dev/ADNLPModels.jl/stable/objects.inv",
        joinpath(@__DIR__, "inventories", "ADNLPModels.toml"),
    ),
    "Tutorials" => (
        "https://control-toolbox.org/Tutorials.jl/stable/",
        "https://control-toolbox.org/Tutorials.jl/stable/objects.inv",
        joinpath(@__DIR__, "inventories", "Tutorials.toml"),
    ),
)

# For reproducibility
mkpath(joinpath(@__DIR__, "src", "assets"))
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

repo_url = "github.com/control-toolbox/OptimalControlProblems.jl"

PROBLEMS_PAGES = [
    joinpath("problems", "beam.md"),
]

makedocs(;
    draft=true, # if draft is true, then the julia code from .md is not executed # debug
    # to disable the draft mode in a specific markdown file, use the following:
    # ```@meta
    # Draft = false
    # ```
    #remotes=nothing,
    warnonly=:cross_references,
    sitename="OptimalControlProblems.jl",
    format=Documenter.HTML(;
        repolink="https://" * repo_url,
        prettyurls=false,
        size_threshold_ignore=["index.md"],
        assets=[
            asset("https://control-toolbox.org/assets/css/documentation.css"),
            asset("https://control-toolbox.org/assets/js/documentation.js"),
        ],
    ),
    pages=[
        "Getting Started" => "index.md",
        "Problems" => [
            "problems-introduction.md",
            "List of the problems" => PROBLEMS_PAGES
        ],
        "Tutorials" => [
            "How to get a problem" => "tutorial-get.md",
            "How to solve a problem" => "tutorial-solve.md",
        ],
        "Developers" => ["How to add a problem" => "dev-add.md"],
    ],
    plugins=[links],
)

deploydocs(; repo=repo_url * ".git", devbranch="main")
