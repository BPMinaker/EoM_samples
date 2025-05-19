module bicycle
using EoM

include(joinpath("models", "input_ex_bicycle_rider.jl"))

format = :screen
# format = :html

vpts = 0:0.08:10

system = [input_ex_bicycle_rider(u=x) for x in vpts]
output = run_eom!.(system, vpts .== 0)
result = analyze.(output, vpts .== 0)

ss = :skip
impulse = :skip
summarize(system, vpts, result; ss, impulse, format)
# summarize(system[10], result[10]; ss, impulse, format)

# using EoM_X3D
# animate_modes(system[10], result[10])

end

println("Done.")