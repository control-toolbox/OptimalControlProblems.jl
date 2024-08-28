using Documenter

makedocs(
    remotes = nothing,
    warnonly = :cross_references,
    sitename = "CTApp",
    format = Documenter.HTML(
        prettyurls = false,
        size_threshold_ignore = ["index.md"],
        assets=[
            asset("https://control-toolbox.org/assets/css/documentation.css"),
            asset("https://control-toolbox.org/assets/js/documentation.js"),
        ],
    ),
    pages = [
        "Introduction" => "index.md",
    ]
)

deploydocs(
    repo = "github.com/control-toolbox/CTApp.git",
    devbranch = "main"
)
