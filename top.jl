using EoM

build_examples()
include(joinpath("examples", "input_ex_top.jl"))
f(x) = input_ex_top(r = x)

vpts= 0:10/50:10
system = f.(vpts)

output = run_eom!.(system, vpts .== 0)
result = analyze.(output, vpts .== 0)

summarize(system, vpts, result; ss = [], vpt_name = ["r" "Angular speed" "rad/s"])

println("Done.")
