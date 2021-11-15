using EoM

build_examples()
include(joinpath("examples", "input_ex_yaw_plane.jl"))

m = 1500
a = 1.5
b = 1.6

f(x) = input_ex_yaw_plane(; u = x, m, a, b)

vpts = 1:0.5:30
system = f.(vpts)
output = run_eom!.(system, vpts .== 1)
result = analyze(output, true)

ss = [1, 2, 3, 4]
bode = [2, 3]
#write_htm5l(system, vpts, result; ss, bode)

summarize(system, vpts, result; ss, bode)


# n = findfirst(vpts .== 20)
# println(result(n))
# println(result(vpts))

println("Done.")
