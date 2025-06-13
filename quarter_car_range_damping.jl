using EoM
include(joinpath("models", "input_ex_quarter_car.jl"))

function main()

    # here you can enter your vehicle specs by name
    ms = 500
    mu = 50
    kt = 150000
    ks = 18000
    vpts = 500:20:3000
    vpt_name = ["c" "Damping" "Ns/m"]

    format = :screen
    # format = :html

    system = [input_ex_quarter_car(; ks, ms, mu, kt, cs=x) for x in vpts]
    output = run_eom!.(system)
    result = analyze.(output)

    ss = :skip
    impulse = :skip
    summarize(system, vpts, result; ss, impulse, vpt_name, format)

    println("Done.")

end

main()
