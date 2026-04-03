using EoM
using Plots
plotlyjs()

format = :screen
#format = :html

include(joinpath("models", "input_ex_smd_wing.jl"))

function main()

    k = 50
    m = 1
    c = 0.2

    system = input_ex_smd_wing(; k, m, c)
    output = run_eom!(system)
    result = analyze(output; ss=:skip)

    summarize(result; format)

end

println("Starting...")
main()
println("Done.")
