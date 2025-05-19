
module disk

using EoM

include(joinpath("models", "input_ex_disk.jl"))

vpts = 0:3/125:3

system = [input_ex_disk(u = x) for x in vpts]
output = run_eom!.(system)
result = analyze.(output; freq=(-1, 1))

ss = :skip
impulse = :skip
summarize(system, vpts, result; ss, impulse)

end

println("Done.")