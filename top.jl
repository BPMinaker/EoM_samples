using EoM

include(joinpath("models", "input_ex_top.jl"))
f(x) = input_ex_top(r=x)

vpts = 0:10/125:10
system = f.(vpts)

output = run_eom!.(system, vpts .== 0)
result = analyze.(output, vpts .== 0)

summarize(system, vpts, result; ss=[], vpt_name=["r" "Angular speed" "rad/s"])
# summarize(system, vpts, result; ss = [], vpt_name = ["r" "Angular speed" "rad/s"], format = :html)

println("Done.")
