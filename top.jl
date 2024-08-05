using EoM

include(joinpath("models", "input_ex_top.jl"))

f(x) = input_ex_top(r = x)
vpts = 0:10/125:10

system = f.(vpts)
output = run_eom!.(system)
result = analyze.(output)

ss = :skip
impulse = :skip
summarize(system, vpts, result; ss, impulse, vpt_name = ["r" "Angular speed" "rad/s"])

println("Done.")
