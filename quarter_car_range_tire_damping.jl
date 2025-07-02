using EoM
include(joinpath("models", "input_ex_quarter_car.jl"))

function main()

    # here you can enter your vehicle specs by name
    ms = 500
    mu = 65
    kt = 200000
    ks = 18000
    cs = 2300
    vpts = 0:10:500
    vpt_name = ["ct" "Tire damping" "Ns/m"]

    format = :screen
    # format = :html

    system = [input_ex_quarter_car(; ks, ms, mu, kt, ct=x, cs) for x in vpts]
    output = run_eom!.(system)
    result = analyze.(output)

    ss = :skip
    impulse = :skip
    summarize(system, vpts, result; ss, impulse, vpt_name, format)

    println("Done.")

end

main()
