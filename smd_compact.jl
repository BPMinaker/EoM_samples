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
    result = analyze(output; ss=:skip)

    ω = 0.95 * result.omega_n[1] * 2π
    u_vec(_, t) = [sin(ω * t)]
    yoft = ltisim(result, u_vec, (0, 10))

    sidx = ["z"]
    p1 = ltiplot(system, yoft; sidx)

    sidx = ["kz", "czdot", "mzddot"]
    p2 = ltiplot(system, yoft; sidx)

    plots = [p1, p2]

    summarize(system, result; plots, format)

    println("Done.")
end

main()
