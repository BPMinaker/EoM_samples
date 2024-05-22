using EoM

include(joinpath("models", "input_ex_rotor.jl"))

vpts = 0.0:2pi/125:2pi
f(x) = input_ex_rotor(; r = x)

system = f.(vpts)
output = run_eom!.(system, vpts .== 0)
result = analyze.(output, vpts .== 0)

bode = [1 0; 1 0; 0 1]
summarize(system, vpts, result; bode, ss = [], vpt_name = ["r" "Angular speed" "rad/s"])
# summarize(system, vpts, result; bode, ss = [], vpt_name = ["r" "Angular speed" "rad/s"], format = :html)

println("Done.")
