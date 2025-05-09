module yaw_plane

using EoM, Interpolations

dpr = 180 / π
vpts = 1:0.2:60

m = 1914
a = 1.473
b = 1.403

Iz = 2600
cf = 2 * 1437 * dpr
cr = 2 * 1507 * dpr

df = 2 * 34 * dpr
dr = 2 * 38 * dpr

ptf = df / cf
ptr = dr / cr

kk=0.5

Eff = kk * 5.80E-5 / dpr
Efr = kk * 1.04E-5 / dpr
Eaf = kk * 2.35E-3 / dpr
Ear = kk * -5.01E-4 / dpr 

lf = Eff / Eaf
kf = Eaf / Eff^2

lr = Efr / Ear
kr = -Ear / Efr^2

muf = 0.1 * b/(a+b) * m
mur = 0.1 * a/(a+b) * m

ms = m - muf - mur
Izs = Iz - muf*a^2 - mur*b^2

include(joinpath("models", "input_ex_yaw_plane_flex_steer.jl"))

f(x) = input_ex_yaw_plane_flex_steer(; u=x, ms, muf, mur, a, b, Izs, cf, cr, ptf, ptr, lf, lr, kf, kr)
system = f.(vpts)
output = run_eom!.(system, vpts .== 1)
result = analyze.(output, vpts .== 1, freq=(-1,2))

summarize(system, vpts, result; impulse = :skip)

ss_resp = hcat(getproperty.(result,:ss_resp)...)
yy = LinearInterpolation(ss_resp[3,:],vpts)
uchar = yy(0.5)
println("Characteristic speed $(my_round(uchar)) m/s.")
K = dpr * (a+b) * 9.81 / uchar^2
println("Understeer gradient $(my_round(K)) degrees/g.")

end

println("Done.")
