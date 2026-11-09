# ==============================================================================
# OptimalControlProblems API Reference Manager
#
# One CTBase automatic_reference_documentation call per documented page (SITE1, SITE2).
# The files of the package are read from `src/<Submodule>/`; keep the lists of the
# extensions in sync with ext/ when files are added, removed, or renamed, otherwise
# docstrings silently drop out of the reference.
#
# The problem functions (`beam`, `chain`, …) are left out: each problem has its own page.
# ==============================================================================

const reference_documentation = CTBase.DevTools.automatic_reference_documentation

"""
    generate_api_reference(src_dir::String, ext_dir::String)

Generate the API reference of OptimalControlProblems and of its extensions (SITE3).
Returns the list of pages.
"""
function generate_api_reference(src_dir::String, ext_dir::String)
    src(files...) = [abspath(joinpath(src_dir, f)) for f in files]
    ext(files...) = [abspath(joinpath(ext_dir, f)) for f in files]

    EXCLUDE_SYMBOLS = vcat(
        Symbol[:include, :eval], collect(OptimalControlProblems.Metadata.PROBLEM_NAMES)
    )

    # The package and its submodules (#413), each with the files of its directory
    # `src/<Name>/`: both lists are read from the package, never written here. The public API
    # of a submodule is what it exports (the package re-exports part of it).
    O = OptimalControlProblems
    submodules = sort!(
        [
            m for m in (getfield(O, n) for n in names(O; all=true)) if
            m isa Module && m !== O && parentmodule(m) === O
        ];
        by=nameof,
    )
    files_of(m) = sort!([
        abspath(f) for f in readdir(joinpath(src_dir, string(nameof(m))); join=true) if
        endswith(f, ".jl")
    ])
    package_modules = Any[
        O => src("OptimalControlProblems.jl");
        [m => files_of(m) for m in submodules]
    ]

    # ── Extensions: their methods extend the package functions, and what is their own is
    #    private, so they appear in the Internals page only (loaded conditionally) ──
    extensions_config = [
        (
            sym=:JuMPModels,
            files=ext(
                "JuMPModels.jl",
                joinpath("JuMPModels", "schemes", "data.jl"),
                joinpath("JuMPModels", "schemes", "tableaux.jl"),
                joinpath("JuMPModels", "schemes", "build.jl"),
            ),
        ),
        (sym=:OptimalControlModels, files=ext("OptimalControlModels.jl")),
        (sym=:ModelingToolkitModels, files=ext("ModelingToolkitModels.jl")),
    ]
    loaded_extensions = [
        (cfg=cfg, mod=Base.get_extension(OptimalControlProblems, cfg.sym)) for
        cfg in extensions_config if
        !isnothing(Base.get_extension(OptimalControlProblems, cfg.sym))
    ]

    # ── Public page: the package ──────────────────────────────────────────────
    pages = Any[reference_documentation(;
        subdirectory="api",
        primary_modules=package_modules,
        exclude=EXCLUDE_SYMBOLS,
        public=true,
        private=false,
        title="OptimalControlProblems",
        title_in_menu="OptimalControlProblems",
        filename="api_public",
    ),]

    # ── Internals: all private symbols in one page, sections by module ────────
    internals_modules = Any[package_modules...]
    for entry in loaded_extensions
        push!(internals_modules, entry.mod => entry.cfg.files)
    end
    push!(
        pages,
        reference_documentation(;
            subdirectory="api",
            primary_modules=internals_modules,
            exclude=EXCLUDE_SYMBOLS,
            public=false,
            private=true,
            title="Internals",
            title_in_menu="Internals",
            filename="internals",
        ),
    )

    return pages
end

"""
    with_api_reference(f::Function, src_dir::String, ext_dir::String)

Generates the API reference, executes `f(pages)`, and cleans up generated files (SITE10).
"""
function with_api_reference(f::Function, src_dir::String, ext_dir::String)
    pages = generate_api_reference(src_dir, ext_dir)
    try
        f(pages)
    finally
        docs_src = abspath(joinpath(@__DIR__, "src"))
        function cleanup(pages)
            for p in pages
                content = last(p)
                if content isa AbstractString
                    fname = endswith(content, ".md") ? content : content * ".md"
                    full_path = joinpath(docs_src, fname)
                    isfile(full_path) && rm(full_path)
                elseif content isa Vector
                    cleanup(content)
                end
            end
        end
        cleanup(pages)
    end
end
