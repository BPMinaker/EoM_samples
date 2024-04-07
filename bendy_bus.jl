using EoM
EoM.plotly()

include(joinpath("models", "input_ex_bendy_bus.jl"))

vpts = 0:800:100000
f(x) = input_ex_bendy_bus(; u = 100 / 3.6, k = x, Γ = 5000)
system = f.(vpts)
output = run_eom!.(system)
result = analyze.(output)

summarize(system, vpts, result; vpt_name = ["k" "Angular stiffness" "Nm/rad"])

println("Done.")
