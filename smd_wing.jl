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
    result = analyze(output; ss=:skip)

    summarize(system, result; format)

end

println("Starting...")
main()
println("Done.")
