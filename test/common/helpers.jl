# Helpers shared by the test environments, transitional: the status-aware loop of the
# registry (#303) replaces them. Included at top level, as `Main.TestHelpers`.
module TestHelpers

using Test: Test

"""
    @broken_if_false e

Record `e` as a pass when it holds, and as *broken* (not as a failure) when it does not.
Returns `true` when `e` holds. Used where a problem is allowed not to converge yet.
"""
macro broken_if_false(e)
    return esc(quote
        let res = Test.@test $e broken = !($e)
            res isa Test.Pass
        end
    end)
end

"""
    broken_if_domain_error(err)

Mark the current test as broken for a `DomainError` raised during a solve (an evaluation
error of the NLP, upstream issue CTDirect#579), and rethrow any other exception.
"""
function broken_if_domain_error(err::Exception)
    err isa DomainError || rethrow(err)
    Test.@test false broken = true
    return nothing
end

end # module TestHelpers
