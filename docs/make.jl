using Documenter

makedocs(;
    remotes=nothing,
    warnonly=:cross_references,
    sitename="OptimalControlProblems",
    format=Documenter.HTML(;
        prettyurls=false,
        size_threshold_ignore=["index.md"],
        assets=[
            asset("https://control-toolbox.org/assets/css/documentation.css"),
            asset("https://control-toolbox.org/assets/js/documentation.js"),
        ],
    ),
    pages=[
        "Introduction" => "index.md",
        "List of the problems" => "list_of_problems.md",
        "Tutorials" => [
            "How to use the models" => "use_models.md",
            "How to solve a problem" => "solve_problem.md",
            ],
        "Developers" => [
            "How to add a new problem" => "add_problem.md",
            ],
    ]
)

deploydocs(;
    repo="github.com/control-toolbox/OptimalControlProblems.jl.git", devbranch="main"
)
