module rotor

using EoM

include(joinpath("models", "input_ex_rotor.jl"))

vpts = 0.0:2pi/125:2pi
f(x) = input_ex_rotor(; r=x)

system = f.(vpts)
output = run_eom!.(system, vpts .== 0)
result = analyze.(output, vpts .== 0; freq=(-2, 1))

bode = [1 0; 1 0; 0 1]
ss = :skip
impulse = :skip
vpt_name = ["r" "Angular speed" "rad/s"]
summarize(system, vpts, result; bode, ss, impulse, vpt_name)
# summarize(system, vpts, result; bode, ss, impulse, vpt_name, format = :html)

end

println("Done.")
