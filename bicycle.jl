using EoM
EoM.plotly()

include(joinpath("models", "input_ex_bicycle_rider.jl"))

f(x) = input_ex_bicycle_rider(u = x)
vpts = 0:0.08:10
system = f.(vpts)

output = run_eom!.(system, vpts .== 0.1)
result = analyze.(output, vpts .== 0.1)

ss = []
summarize(system, vpts, result; ss)
# summarize(system[10], result[10]; ss, format = :html)

# using EoM_X3D
# animate_modes(system[10], result[10])

println("Done.")