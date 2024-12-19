module yaw_plane

using EoM, Interpolations


dpr = 180 / π
vpts = 1:0.2:60

mt = 1914
a = 1.473
b = 1.403
Izt = 2600
cf = 2 * 1437 * dpr
cr = 2 * 1507 * dpr

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


muf = 0.1 * b/(a+b) * mt
mur = 0.1 * a/(a+b) * mt

ms = mt -muf - mur

c = (b * mur - a * muf) / ms

Iz = Izt - muf*a^2 -mur*b^2 - ms*c^2


include(joinpath("models", "input_ex_yaw_plane_flex_steer.jl"))

f(x) = input_ex_yaw_plane_flex_steer(; u=x, ms, muf, mur, a, b, c, Iz, cf, cr, ptf, ptr, lf, lr, kf, kr)
system = f.(vpts)
output = run_eom!.(system, vpts .== 1)
result = analyze.(output, vpts .== 1, freq=(-1,2))

ss = [1, 1, 1, 1, 0, 0, 1, 1, 1, 1]


summarize(system, vpts, result; ss, impulse = :skip, bode = :skip)

ss_resp = hcat(getproperty.(result,:ss_resp)...)
yy = LinearInterpolation(ss_resp[3,:],vpts)
uchar = yy(0.5)
display(uchar)
K = dpr * (a+b) * 9.81 / uchar^2
display(K)


# using EoM_X3D
#eom_draw(system[1])
#animate_modes(system[end], result[end], scale=0.2)

end

println("Done.")
