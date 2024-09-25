using EoM

include(joinpath("models", "input_ex_bicycle_rider.jl"))

f(x) = input_ex_bicycle_rider(u=x)
vpts = 0:0.08:10
system = f.(vpts)

output = run_eom!.(system, vpts .== 0)
result = analyze.(output, vpts .== 0)

ss = :skip
impulse = :skip
summarize(system, vpts, result; ss, impulse)
# summarize(system[10], result[10]; ss, impulse, format = :html)

# using EoM_X3D
# animate_modes(system[10], result[10])

println("Done.")