module yaw_plane

using EoM, Interpolations

g=9.81

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


ef = 0.11 # steer to roll ratio
er = -0.03
k_phi = 0.00482 # rad / m/s^2 roll flexibility


ccf = 114 * dpr
ccr = 122 * dpr


wf = b/(a+b)*mt*g
wr = a/(a+b)*mt*g 





k_us_f0 = wf/cf * (1 + dr/2/b/(cr/2))
k_us_r0 = wr/cr * (1 - df/2/a/(cf/2))

display(k_us_f0)
display(k_us_r0)

display(k_us_f0 - k_us_r0)
display((k_us_f0 - k_us_r0) * dpr)

println("")

k_us_f1 = wf/cf * (1 + Eaf*df/2) * (1 + dr/2/b/(cr/2))
k_us_r1 = wr/cr * (1 - Ear*dr/2) * (1 - df/2/a/(cf/2))

display(k_us_f1)
display(k_us_r1)

display(k_us_f1 - k_us_r1)


k_us_f2 = Eff * wf * 0.9 / 2
k_us_r2 = Efr * wr * 0.9 / 2


display(k_us_f2)
display(k_us_r2)

display(k_us_f2 + k_us_r2)
println("")



k_us_f3 = k_phi * g * ef
k_us_r3 = k_phi * g * er



display(k_us_f3)
display(k_us_r3)

display(k_us_f3 + k_us_r3)
println("")



k_us_f4 = k_phi * g * (1 + Eaf*df/2) * ef/(cf/2) * ccf
k_us_r4 = k_phi * g * (1 - Ear*dr/2) * er/(cr/2) * ccr


display(k_us_f4)
display(k_us_r4)

display(k_us_f4 + k_us_r4)
println("")



dsfdsdsf()


include(joinpath("models", "input_ex_yaw_plane_flex_roll_steer.jl"))

f(x) = input_ex_yaw_plane_flex_roll_steer(; u=x, ms, muf, mur, a, b, c, Iz, cf, cr, ptf, ptr, lf, lr, kf, kr, ef, er, k_phi)
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
