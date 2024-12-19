module yaw_plane

using EoM, Interpolations


dpr = 180 / π

m = 1914
a = 1.473
b = 1.403
Iz = 2600
cf = 2 * 1437 * dpr
cr = 2 * 1507 * dpr

vpts = 1:0.5:70


ef = 0.11 # steer to roll ratio
er = -0.03
k_phi = 0.00482 # rad / m/s^2 roll flexibility

include(joinpath("models", "input_ex_yaw_plane_roll_steer.jl"))

f(x) = input_ex_yaw_plane_roll_steer(; u=x, m, a, b, Iz, cf, cr, ef, er, k_phi)
system = f.(vpts)
output = run_eom!.(system, vpts .== 1)
result = analyze.(output, vpts .== 1, freq=(-1,2))

ss = [1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1]


summarize(system, vpts, result; ss, impulse = :skip, bode = :skip)

ss_resp = hcat(getproperty.(result,:ss_resp)...)
yy = LinearInterpolation(ss_resp[3,:],vpts)
uchar = yy(0.5)
display(uchar)
K = dpr * (a+b) * 9.81 / uchar^2
display(K)

end

println("Done.")
