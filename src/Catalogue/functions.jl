# Generic methods are installed before weak-dependency extensions load.
for problem in Metadata.PROBLEM_NAMES
    doc = """Return the `$problem` problem for a selected modeling.

Available modelings and statuses are listed by `OptimalControlProblems.status(:$problem)`."""
    code = quote
        @doc $doc function $(problem)(
            model_backend::T, args...; kwargs...
        ) where {T<:Modelings.AbstractModelBackend}
            spec = Modelings._modeling_spec(model_backend)
            if !Modelings._problem_exists($(QuoteNode(problem)), spec)
                throw(
                    CTBase.Exceptions.IncorrectArgument(
                        string(
                            $(QuoteNode(problem)),
                            " is not available for ",
                            spec.name,
                            "Backend; see problems(",
                            spec.name,
                            "Backend())",
                        ),
                    ),
                )
            end
            _warn_if_draft($(QuoteNode(problem)), spec.name)
            throw(CTBase.Exceptions.ExtensionError(Modelings.WEAKDEPS[T]))
        end
    end
    eval(code)
end
