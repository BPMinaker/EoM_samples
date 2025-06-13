using EoM
include(joinpath("models", "input_ex_smd.jl"))

function main()

    format = :screen
    #format = :html

    k = 50
    m = 1
    c = 0.2

    system = input_ex_smd(; k, m, c)
    output = run_eom!(system)
    result = analyze(output)

    ω = 0.95 * result.omega_n[1] * 2π
    u_vec(_, t) = [sin(ω * t)]
    yoft = ltisim(result, u_vec, (0, 10))

    yidx = [1]
    p1 = ltiplot(system, yoft; yidx)

    yidx = [2, 3, 4]
    p2 = ltiplot(system, yoft; yidx)

    plots = [p1, p2]

    summarize(system, result; ss=:skip, plots, format)

    println("Done.")
end

main()
