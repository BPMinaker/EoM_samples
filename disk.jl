using EoM
EoM.plotly()

include(joinpath("models", "input_ex_disk.jl"))

f(x) = input_ex_disk(u = x)
vpts = 0:3/125:3

system = f.(vpts)
output = run_eom!.(system)
result = analyze.(output)

summarize(system, vpts, result)

println("Done.")


