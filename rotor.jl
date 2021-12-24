using EoM

build_examples()
include(joinpath("examples", "input_ex_rotor.jl"))

vpts = 0.0:2pi/50:2pi
f(x) = input_ex_rotor(; r = x)

system = f.(vpts)
output = run_eom!.(system, vpts .== 0)
result = analyze(output, true)


bode = [1 0; 1 0; 0 1]
summarize(system, vpts, result; bode, ss = [], vpt_name = ["r" "Angular speed" "rad/s"])

# write_html(system, vpts, result, ss = [], vpt_name = ["r" "Angular speed" "rad/s"])

println("Done.")
