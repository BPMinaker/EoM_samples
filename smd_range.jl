using EoM, Polynomials
include(joinpath("models", "input_ex_smd.jl"))
# this file repeats the spring mass damper example, but shows how we can analyze a series of systems, using Julia's dot notation

function main()

    format = :screen
    # format = :html

    # set stiffness and mass so that ω_n is 1 Hz (2π rad/s), note that ζ = 1 (critical damping) is therefore c = 4π (from ζ = c/2√km)
    k = 4π^2
    m = 1.0
    c_cr = 2 * (k * m)^0.5

    # then we define the range of values for c, from 0 to 1.5 times critical
    vpts = range(0, 1.5 * c_cr; length=151)
    vpt_name = ["c" "Damping coefficient" "N/(m/s)"]

    # we generate our vector of systems, one entry for each value of c
    system = [input_ex_smd(; k, m, c=x) for x in vpts]

    # we take the vector of input systems, and generate the equations for all of them, using dot notation
    output = run_eom!.(system)

    # we take the vector of output equations, and analyze all of them, again using dot notation
    impulse = [1, 0, 0, 0]
    result = analyze.(output; impulse, ss=:skip)

    # now, just for fun, let's plot ms^2 + cs + k over a range of s, to confirm how the roots align with our eigenvalues
    xlabel = "s [rad/s]"
    ylabel = "p(s) [N/m]"
    p1 = EoM.plot(; xlabel, ylabel)

    s = range(-20, 5; length=201)
    for i in vpts[1:15:end]
        label = "c = $(my_round(i))"
        p = Polynomial([k, i, m])
        EoM.plot!(p1, s, p.(s); label)
    end
    plots = [p1]

    # the `summarize()` function has been written using another feature of Julia, called `multiple dispatch`, which allows the same function to do different things, depending on the type of arguments, `summarize()` recognizes if `system` and `result` are vectors, and if so, it drops the tables, and gives a series of plots instead

    summarize(system, vpts, result; vpt_name, plots, format)

end

println("Starting...")
main()
println("Done.")
