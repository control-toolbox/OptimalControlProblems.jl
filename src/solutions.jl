# Analytic solutions of the continuous problems (#379), used by the `:reference_solution` of
# their metadata. Each returns `nothing` for the parameters it is not known for.

# The Bryson–Denham problem (Bryson, Denham & Dreyfus 1963), shared by `beam` and
# `bryson_denham`: ẋ₁ = x₂, ẋ₂ = u on [0, 1], x(0) = (0, 1), x(1) = (0, -1), x₁ ≤ ℓ, cost
# `weight * ∫u²`. Known here for ℓ ≤ 1/6, where x₁ = ℓ on the boundary arc [3ℓ, 1 - 3ℓ].
function _bryson_denham_solution(t0, tf, x_t0, x_tf, ℓ; weight, lower=(-Inf))
    known = (t0, tf, x_t0, x_tf) == (0, 1, (0, 1), (0, -1)) && 0 < ℓ ≤ 1 / 6 && lower ≤ 0
    known || return nothing
    a = 3ℓ
    # distance to the nearest end of [0, 1], and the sign of x₂ there
    side(t) = t ≤ 1 / 2 ? (t, 1) : (1 - t, -1)
    r(t) = max(1 - first(side(t)) / a, 0)
    return (
        state=t -> [ℓ * (1 - r(t)^3), last(side(t)) * r(t)^2],
        control=t -> -2 * r(t) / a,
        objective=weight * 8 / (9ℓ),
        source="Bryson, Denham & Dreyfus (1963), https://doi.org/10.2514/3.2107",
    )
end
