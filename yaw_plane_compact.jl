using EoM

build_examples()
include(joinpath("examples", "input_ex_yaw_plane.jl"))

m = 1500
a = 1.5
b = 1.6
vpts = 1:0.5:30

f(x) = input_ex_yaw_plane(; u = x, m, a, b)
system = f.(vpts)
output = run_eom!.(system, vpts .== 1)
result = analyze(output, true)

ss = [1, 1, 1, 1, 0, 0, 0]
bode = [0, 1, 1, 0, 0, 0, 1]
summarize(system, vpts, result; ss, bode)

println("Done.")
