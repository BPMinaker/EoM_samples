using EoM
using Interpolations
using Plots
plotlyjs()

format = :screen
# format = :html

include(joinpath("models", "input_ex_yaw_plane.jl"))

function main()

    # here you can enter your vehicle specs by name, including m, Iz, a, b, cf, cr; make sure you add the property you want to set to the argument list of `input_ex_yaw_plane()` below after you set it; properties you don't set will use defaults defined in `input_ex_yaw_plane()`

    dpr = 180 / π

    # here we set the speed in `vpts`, which gets sent one at a time to the `input_ex_yaw_plane()` function, where they determine the value of `u`, the forward speed
    vpts = 0.4:0.4:40

    m = 1914 # mass
    a = 1.473 # front wheelbase
    b = 1.403 # rear wheelbase
    Iz = 2600 # inertia
    cf = 2 * 1437 * dpr # front axle cornering stiffness in N/rad
    cr = 2 * 1507 * dpr # rear axle cornering stiffness in N/rad

    df = 2 * 34 * dpr # front axle self-aligning moment stiffness in Nm/rad
    dr = 2 * 38 * dpr # rear axle self-aligning moment stiffness in Nm/rad

    ptf = df / cf # front pneumatic trail
    ptr = dr / cr # rear pneumatic trail

    # generate over a range of speeds to find characteristic speed
    vpts = 120:0.1:130
    system = [input_ex_yaw_plane(; u=x, m, a, b, Iz, cf, cr, ptf, ptr) for x in vpts]
    output = run_eom!.(system)
    result = analyze.(output; freq=(-1, 1), bode = :skip, impulse = :skip)

    sidx = system[1].sidx["α_u"]
    ss_resp = hcat(getproperty.(result, :ss_resp)...)
    α_u = ss_resp[sidx, :]
    if  minimum(α_u) < 0.5 && maximum(α_u) > 0.5
        yy = LinearInterpolation(α_u, vpts)
        u_char = yy(0.5)
        println("Characteristic speed $(my_round(u_char)) m/s.")
        K = dpr * (a + b) * 9.81 / u_char^2
        println("Understeer gradient $(my_round(K)) degrees/g.")

        # characteristic speed can also be calculated from the vehicle parameters, as follows:
        u_char_calc = sqrt((a-ptf + b+ptr)^2 * cf * cr / (m * (cr * (b+ptr) - cf * (a-ptf))))
        println("Characteristic speed calculated from vehicle parameters $(my_round(u_char_calc)) m/s.")

    else
        println("Characteristic speed not found in range.  Try a larger range.")

    end
end

println("Starting...")
main()
println("Done.")
