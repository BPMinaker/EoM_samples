using EoM, EoM_X3D
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

    format = :screen
    # format = :html

    system = input_ex_yaw_plane(; u, m, a, b, Iz, cf, cr, ptf, ptr)
    sensors_animate!(system)
    output = run_eom!(system)
    result = analyze(output; ss = :skip, impulse = :skip, bode = :skip)

    #define the steer angle as a function of time, a sin w dwell input ala FMVSS 126
    # a 0.7 Hz sinewave with origin at t=2 times zero everywhere except times one from t=2 for 3/4 of a wavelength
    # plus a constant negative one for 0.5 seconds,starting right after the 3/4 wavelength
    # plus a 0.7 Hz sinewave with origin at t=2.5 times zero everywhere except times one for the last 1/4 of a wavelength
    # all times 2
#    steer(t) =  2 * (sin(2π * 0.7 * (t - 2)) * EoM.pulse(t, 2, 2 + 0.75 / 0.7) - EoM.pulse(t, 2 + 0.75 / 0.7, 2.5 + 0.75 / 0.7) + sin(2π * 0.7 * (t - 2.5)) * EoM.pulse(t, 2.5 + 0.75 / 0.7, 2.5 + 1 / 0.7))

    # define input function to be steer
    # put it in the form to also accept x (i.e., u=f(x,t))) but then ignore x
    # and return a vector even though it is length of one
    function u_vec(x, t)

        y = result.ss_eqns.C * x
        # get vehicle location and heading from sensors (y is the output vector)
        offset = y[system.sidx["chassis_2"]]
        heading = y[system.sidx["chassis_6"]]
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
    p1 = ltiplot(system, yoft; sidx)

    # plot body slip angle vs time
    sidx = ["β"]
    p2 = ltiplot(system, yoft; sidx)

    # plot slip angles, understeer angle vs time
    sidx = ["α_f", "α_r", "α_u"]
    p3 = ltiplot(system, yoft; sidx)

    # plot lateral acceleration vs time
    sidx = ["a_y"]
    p4 = ltiplot(system, yoft; sidx)

    # plot path, noting that it is not even close to uniform scaling, x ~ 400 m, y ~ 2.5 m
    # becasue this plot is not a function of time, we need to use the EoM.plot function
    xlabel = "x [m]"
    ylabel = "y [m]"
    label = ["Path" "Target path"]
    lw = 2 # thicker line weight
    yidx= system.sidx["y"]
    size = (800, 400)

    x = u * yoft.t
    track_y(x) = track(x)[1]
    path = track_y.(x)

    p5 = EoM.plot(x, [yoft[yidx, :] path]; xlabel, ylabel, label, lw, size)

    plots = [p1, p2, p3, p4, p5]

    # write all the results; steady state plots of outputs 1 through 4, 7, 8 (5 and 6 don't reach steady state)
    summarize(system, result; plots, format)

    println("Done.")

end

main()
