using EoM

include(joinpath("models", "input_ex_yaw_plane.jl"))

m = 1600
fwf = 0.58
wb = 2.6
b = wb * fwf
a = wb - b
Iz = 2600
cf = 70000
cr = 80000
vpts = 1:0.5:30

f(x) = input_ex_yaw_plane(; u=x, m, a, b, Iz, cf, cr)
system = f.(vpts)
output = run_eom!.(system, vpts .== 1)
result = analyze.(output, vpts .== 1)

ss = [1, 1, 1, 1, 0, 0, 1, 1]
bode = [0, 1, 1, 0, 0, 1, 1, 1]
summarize(system, vpts, result; ss, bode)
summarize(system, vpts, result; ss, bode, format = :html)

# n = findfirst(vpts .== 20)
# summarize(system[n], result[n]; ss, bode)

println("Done.")
