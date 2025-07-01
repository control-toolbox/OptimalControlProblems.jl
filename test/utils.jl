function costateInterpolation(p, t)
    nx = size(p[1])[1]
    n_h = length(t)[1]
    res = [zeros(Float64, nx) for _ in 1:n_h]
    for j in 1:nx
        pj = Interpolations.linear_interpolation(t[1:end-1], [p[i][j] for i in 1:n_h-1], extrapolation_bc=Interpolations.Line())
        f = t -> pj(t)
        for i in 1:n_h
            res[i][j] = f(t[i])
        end
    end
    return res
end

function prettytime(t)
    if t < 1e3
        value, units = t, "ns"
    elseif t < 1e6
        value, units = t / 1e3, "μs"
    elseif t < 1e9
        value, units = t / 1e6, "ms"
    else
        value, units = t / 1e9, "s"
    end
    return string(value, " ", units)
end

### compute norm_Lp for u : R → R

function norm_Lp(u, p, dt)
    if p == Inf
        if isa(u[1], Number)
            return maximum(abs, u)
        else
            return maximum([maximum(abs, ui) for ui in u])
        end
    end
    nu = length(u)
    if nu < 2
        return (sum(abs.(u[1]) .^ p) * dt)^(1 / p)
    end
    s = 0.0
    for i in 1:nu-1
        s += 0.5 * (sum(abs.(u[i]) .^ p) + sum(abs.(u[i+1]) .^ p))
    end
    return (s * dt)^(1 / p)
end

function available_problems()
    cache_file = joinpath(@__DIR__, "..", "available_problems_cache.txt")
    
    if isfile(cache_file)
        try
            content = read(cache_file, String)
            if !isempty(strip(content))
                # Parse symbols from the file
                lines = split(strip(content), '\n')
                return [Symbol(strip(line)) for line in lines if !isempty(strip(line))]
            end
        catch e
            @warn "Error reading the cache: $e"
        end
    end
    
    # Default list if the cache does not exist or is empty
    @warn "Available problems cache not found. Run the tests to update the list."
    return Symbol[]
end