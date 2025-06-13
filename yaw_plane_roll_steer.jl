
using EoM, Interpolations

include(joinpath("models", "input_ex_yaw_plane_roll_steer.jl"))

function main()

    g = 9.81
    dpr = 180 / π
    vpts = 1:0.2:80

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

    ef = 0.11 # steer to roll ratio
    er = 0.03

    k_phi = (2.71 / dpr) / g  # rad / m/s^2 roll flexibility

    system = [input_ex_yaw_plane_roll_steer(; u=x, m, a, b, Iz, cf, cr, ef, er, k_phi, ptf, ptr) for x in vpts]
    output = run_eom!.(system, vpts .== 1)
    result = analyze.(output, vpts .== 1, freq=(-1, 2))

    ss = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
    summarize(system, vpts, result; ss, impulse=:skip, bode=:skip)

    ss_resp = hcat(getproperty.(result, :ss_resp)...)
    yy = LinearInterpolation(ss_resp[3, :], vpts)
    uchar = yy(0.5)
    println("Characteristic speed $(my_round(uchar)) m/s.")
    K = dpr * (a + b) * 9.81 / uchar^2
    println("Understeer gradient $(my_round(K)) degrees/g.")

    println("Done.")

end

main()
