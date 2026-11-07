# The files that define `problem`: its metadata, its description and its model in each
# modeling where it exists. Its reference objectives are not part of it: they are computed
# from its results.
function _problem_files(problem::Symbol)
    files = [
        joinpath(Modelings.EXT_PATH, "MetaData", "$(problem).jl"),
        joinpath(Modelings.EXT_PATH, "Descriptions", "$(problem).md"),
    ]
    for spec in Modelings.MODELING_SPECS
        Modelings._problem_exists(problem, spec) &&
            push!(files, Modelings._problem_file(problem, spec))
    end
    return files
end

"""
Fingerprint of `problem` (#314): the SHA-256 of the files that define it (metadata,
description, model files), recorded with each result and compared by the documentation to
tell whether a result is up to date. The same on every platform: paths are relative to the
package and written with `/`, and line endings are read as `\n`.
"""
function _fingerprint(problem::Symbol)
    return _fingerprint(_problem_files(problem), dirname(Modelings.EXT_PATH))
end

function _fingerprint(files, root::AbstractString)
    ctx = SHA.SHA256_CTX()
    for file in files
        SHA.update!(ctx, codeunits(join(splitpath(relpath(file, root)), "/") * "\0"))
        SHA.update!(ctx, codeunits(replace(read(file, String), "\r\n" => "\n") * "\0"))
    end
    return bytes2hex(SHA.digest!(ctx))
end
