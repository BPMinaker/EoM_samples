using EoM, EoM_X3D
using Plots
plotlyjs()

format = :screen
# format = :html

include(joinpath("models", "input_ex_yaw_plane.jl"))
include(joinpath("models", "driver.jl"))
include(joinpath("models", "track.jl"))

function main()

    # here you can enter your vehicle specs by name, including m, Iz, a, b, cf, cr; make sure you add the property you want to set to the argument list of `input_ex_yaw_plane()` below after you set it; properties you don't set will use defaults defined in `input_ex_yaw_plane()`

    dpr = 180 / π

    u = 30

    m = 1914 # mass
    a = 1.473 # front wheelbase
    b = 1.503 # rear wheelbase
    Iz = 2600 # inertia
    cf = 2 * 1437 * dpr # front axle cornering stiffness in N/rad
    cr = 2 * 1507 * dpr # rear axle cornering stiffness in N/rad

    df = 2 * 34 * dpr # front axle self-aligning moment stiffness in Nm/rad
    dr = 2 * 38 * dpr # rear axle self-aligning moment stiffness in Nm/rad

    ptf = df / cf # front pneumatic trail
    ptr = dr / cr # rear pneumatic trail

    system = input_ex_yaw_plane(; u, m, a, b, Iz, cf, cr, ptf, ptr)
    sensors_animate!(system)
    output = run_eom!(system)
    result = analyze(output; ss = :skip, impulse = :skip, bode = :skip)

    # define input function to be steer
    function u_vec(x, t)

        y = result.ss_eqns.C * x
        # get vehicle location and heading from sensors (y is the output vector)
        offset = y[system.sidx["y"]]
        heading = y[system.sidx["ψ"]] / dpr 
        [-dpr * driver(a+b, offset, heading, u * t)]
    end

    # define time interval
    t1 = 0
    t2 = 10
    yoft = ltisim(result, u_vec, (t1, t2))

    animate_history(system, yoft)

    # notation conflict, y is system output vector, but also lateral displacement
    # sensors are, in order, r, β, α_u, a_lat, y, θ, α_f, α_r

    # plot yaw rate vs time
    sidx = ["r"]
    p1 = ltiplot(yoft; sidx)

    # plot body slip angle vs time
    sidx = ["β"]
    p2 = ltiplot(yoft; sidx)

    # plot slip angles, understeer angle vs time
    sidx = ["α_f", "α_r", "α_u"]
    p3 = ltiplot(yoft; sidx)

    # plot lateral acceleration vs time
    sidx = ["a_y"]
    p4 = ltiplot(yoft; sidx)

    # plot path, noting that it is not even close to uniform scaling, x ~ 400 m, y ~ 2.5 m
    # becasue this plot is not a function of time, we need to use the EoM.plot function
    xlabel = "x [m]"
    ylabel = "y [m]"
    label = ["Path" "Target path"]
    yidx= system.sidx["y"]

    x = u * yoft.t
    track_y(x) = track(x)[1]
    path = track_y.(x)

    p5 = plot(x, [yoft[yidx, :] path]; xlabel, ylabel, label)

    plots = [p1, p2, p3, p4, p5]

    # write all the results; steady state plots of outputs 1 through 4, 7, 8 (5 and 6 don't reach steady state)
    summarize(result; plots, format)

end

println("Starting...")
main()
println("Done.")
