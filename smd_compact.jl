using EoM
using Plots
plotlyjs()

format = :screen
#format = :html

include(joinpath("models", "input_ex_smd.jl"))

function main()

    k = 50
    m = 3
    c = 0.2

    system = input_ex_smd(; k, m, c)
    output = run_eom!(system)
    result = analyze(output; ss=:skip)

    ω = 0.95 * result.omega_n[1] * 2π
    u_vec(_, t) = [sin(ω * t)]
    yoft = ltisim(result, u_vec, (0, 10))

    plots = [ltiplot(yoft; sidx=i) for i in [["z"], ["kz", "czdot", "mzddot"]]]

    summarize(result; plots, format, tex=true)

end

println("Starting...")
main()
println("Done.")
