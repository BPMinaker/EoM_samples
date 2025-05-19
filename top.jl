module top

using EoM

include(joinpath("models", "input_ex_top.jl"))

vpts = 0:10/125:10

system = [input_ex_top(r=x) for x in vpts]
output = run_eom!.(system)
result = analyze.(output)

ss = :skip
impulse = :skip
vpt_name = ["r" "Angular speed" "rad/s"]
summarize(system, vpts, result; ss, impulse, vpt_name)

end

println("Done.")
