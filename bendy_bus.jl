using EoM
include(joinpath("models", "input_ex_bendy_bus.jl"))

function main()

    vpts = 0:800:100000
    vpt_name = ["k" "Angular stiffness" "Nm/rad"]

    u = 100 / 3.6
    Γ = 5000

    system = [input_ex_bendy_bus(; u, k=x, Γ) for x in vpts]
    output = run_eom!.(system)
    result = analyze.(output; ss=:skip, impulse=:skip)

    summarize(system, vpts, result; vpt_name)

end

println("Starting...")
main()
println("Done.")
