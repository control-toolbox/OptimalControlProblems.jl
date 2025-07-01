function test_aqua()

    println()
    println("\033[1m###########################\033[0m")
    println("\033[1m######## TEST AQUA ########\033[0m")
    println("\033[1m###########################\033[0m")
    println()
    @testset "Aqua.jl" begin
        Aqua.test_all(
            OptimalControlProblems;
            ambiguities=false,
            #stale_deps=(ignore=[:SomePackage],),
            deps_compat=(ignore=[:LinearAlgebra, :Unicode],),
            piracies=true,
        )
        # do not warn about ambiguities in dependencies
        Aqua.test_ambiguities(OptimalControlProblems)
    end

    println()
    println("\033[1m###########################\033[0m")
    println("\033[1m##### END TEST AQUA #######\033[0m")
    println("\033[1m###########################\033[0m")
    println()
end
