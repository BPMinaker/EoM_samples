using EoM
include(joinpath("models", "input_ex_quarter_car.jl"))

function main()

    # here you can enter your vehicle specs by name
    ms = 500
    mu = 65
    kt = 200000
    cs = 1500
    ct = 100
    vpts = 15000:100:30000
    vpt_name = ["k" "Stiffness" "N/m"]

    format = :screen
    # format = :html

    system = [input_ex_quarter_car(; ks=x, ms, mu, kt, cs, ct) for x in vpts]
    output = run_eom!.(system)
    result = analyze.(output; ss = :skip, impulse = :skip)
    summarize(system, vpts, result; vpt_name, format)

end

println("Starting...")
main()
println("Done.")
