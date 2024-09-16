using EoM

m = 1914
a = 1.473
b = 1.403
Iz = 2600
cf = 2 * 1437 * 180 / π
cr = 2 * 1507 * 180 / π

vpts = 1:0.3:40

include(joinpath("models", "input_ex_yaw_plane.jl"))
f(x) = input_ex_yaw_plane(; u=x, m, a, b, Iz, cf, cr)
system = f.(vpts)
output = run_eom!.(system, vpts .== 1)
result = analyze.(output, vpts .== 1, freq=(-1,2))

ss = [1, 1, 1, 1, 0, 0, 1, 1]
impulse = :skip
summarize(system, vpts, result; ss, impulse)
#summarize(system, vpts, result; ss, impulse, format = :html)

# n = argmin(abs.(vpts .- 20))
# summarize(system[n], result[n]; ss)

Eff = -5.8E-5 * π / 180
Efr = 1.04E-5 * π / 180
Eaf = 2.35E-3 * π / 180
Ear = 5.01E-4 * π / 180

lf = Eff / Eaf
kf = Eaf / Eff^2

lr = Efr / Ear
kr = Ear / Efr^2

include(joinpath("models", "input_ex_yaw_plane_flex.jl"))

f(x) = input_ex_yaw_plane_flex(; u=x, m, a, b, Iz, cf, cr, lf, lr, kf, kr)
system = f.(vpts)
output = run_eom!.(system, vpts .== 1)
result = analyze.(output, vpts .== 1, freq=(-1,2))

summarize(system, vpts, result; ss, impulse)

using EoM_X3D
#eom_draw(system[1])
animate_modes(system[end], result[end], scale=0.2)


println("Done.")
