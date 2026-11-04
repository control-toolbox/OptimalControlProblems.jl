# Test options shared by every test file (TST1): overridable verbosity and timing.
# Included at top level by the dispatcher and by each environment, so that
# `Main.TestOptions` exists when a test file looks for it.
module TestOptions
const VERBOSE = true
const SHOWTIMING = true
end
