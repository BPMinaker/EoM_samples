module yaw_plane

using EoM


dpr = 180 / π

m = 1914
a = 1.473
b = 1.403
Iz = 2600
cf = 2 * 1437 * dpr
cr = 2 * 1507 * dpr

vpts = 1:0.1:60

#=
include(joinpath("models", "input_ex_yaw_plane.jl"))
f(x) = input_ex_yaw_plane(; u=x, m, a, b, Iz, cf, cr)
system = f.(vpts)
output = run_eom!.(system, vpts .== 1)
result = analyze.(output, vpts .== 1, freq=(-1,2))


ss = [1, 1, 1, 1, 0, 0, 1, 1]
impulse = :skip
summarize(system, vpts, result; ss, impulse)
#summarize(system, vpts, result; ss, impulse, format = :html)

ss_resp = hcat(getproperty.(result,:ss_resp)...)
ua = ss_resp[3,:]
~,ind = findmin(abs.(ua .- 0.5))
uchar = vpts[ind]
display(uchar)
K = dpr * (a+b) * 9.81 / uchar^2
display(K)
=#


# n = argmin(abs.(vpts .- 20))
# summarize(system[n], result[n]; ss)


df = 2 * 34 * dpr
dr = 2 * 38 * dpr

ptf = df / cf
ptr = dr / cr


Eff = -5.8E-5 / dpr
Efr = 1.04E-5 / dpr
Eaf = 2.35E-3 / dpr
Ear = 5.01E-4 / dpr

lf = Eff / Eaf
kf = 2 * Eaf / Eff^2

lr = Efr / Ear
kr = 2 * Ear / Efr^2

include(joinpath("models", "input_ex_yaw_plane_flex.jl"))

f(x) = input_ex_yaw_plane_flex(; u=x, m, a, b, Iz, cf, cr, ptf, ptr, lf, lr, kf, kr)
system = f.(vpts)
output = run_eom!.(system, vpts .== 1)
result = analyze.(output, vpts .== 1, freq=(-1,2))

ss = [1, 1, 1, 1, 0, 0, 1, 1, 1, 1]


summarize(system, vpts, result; ss, impulse = :skip, bode = :skip)

ss_resp = hcat(getproperty.(result,:ss_resp)...)
ua = ss_resp[3,:]
~,ind = findmin(abs.(ua .- 0.5))
uchar = vpts[ind]
display(uchar)
K = dpr * (a+b) * 9.81 / uchar^2
display(K)

# using EoM_X3D
#eom_draw(system[1])
#animate_modes(system[end], result[end], scale=0.2)

end

println("Done.")
