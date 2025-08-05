using EoM
include(joinpath("models", "input_ex_smd_wing.jl"))

function main()

    format = :screen
    #format = :html

    k = 50
    m = 1
    c = 0.2

    system = input_ex_smd_wing(; k, m, c)
    output = run_eom!(system)
    result = analyze(output)

    summarize(system, result; ss=:skip, format)

    println("Done.")
end

main()
