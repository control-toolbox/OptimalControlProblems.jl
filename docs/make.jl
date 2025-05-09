using Documenter

cp("./docs/Manifest.toml", "./docs/src/assets/Manifest.toml", force = true)
cp("./docs/Project.toml", "./docs/src/assets/Project.toml", force = true)

repo_url = "github.com/control-toolbox/OptimalControlProblems.jl"

makedocs(;
    remotes=nothing,
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
        "List of the Problems" => "list_of_problems.md",
        "Tutorials" => [
            "How to get a problem" => "use_models.md",
            "How to solve a problem" => "solve_problem.md",
        ],
        "Developers" => ["How to add a problem" => "add_problem.md"],
    ],
)

deploydocs(; repo=repo_url * ".git", devbranch="main")
