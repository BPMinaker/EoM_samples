using EoM
using Interpolations
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

    format = :screen
    # format = :html

    # generate our vector of systems
    system = [input_ex_yaw_plane(; u=x, m, a, b, Iz, cf, cr, ptf, ptr) for x in vpts]

    # generate the equations of motion, but many times, for every different value of forward speed
    output = run_eom!.(system)

    # do the eigenvalues, freq resp, etc, for each forward speed
    result = analyze.(output; freq=(-1, 1))

    # sensors are, in order, r, β, α_u, a_lat, y, θ, α_f, α_r
    # write all the results; steady state plots of outputs 1 through 4, 7, 8 (5 and 6 don't reach steady state)
    ss = [1, 1, 1, 1, 0, 0, 1, 1]
    impulse = :skip
    bode = :skip
    summarize(system, vpts, result; ss, impulse, bode, format)

    # now, let's also do some time domain solutions; let's pick a speed of 100 km/h, or 22.8 m/s, and get the equations of motion and the results for that speed
    u = 22.8
    n = findfirst(vpts .== u)
    system = system[n]
    system.name *= " $u m per s"
    result = result[n]

    #define the steer angle as a function of time, a sin w dwell input ala FMVSS 126
    # a 0.7 Hz sinewave with origin at t=2 times zero everywhere except times one from t=2 for 3/4 of a wavelength
    # plus a constant negative one for 0.5 seconds,starting right after the 3/4 wavelength
    # plus a 0.7 Hz sinewave with origin at t=2.5 times zero everywhere except times one for the last 1/4 of a wavelength
    # all times 2
    steer(t) = 2 * (
        sin(2π * 0.7 * (t - 2)) * EoM.pulse(t, 2, 2 + 0.75 / 0.7)
        -
        EoM.pulse(t, 2 + 0.75 / 0.7, 2.5 + 0.75 / 0.7)
        +
        sin(2π * 0.7 * (t - 2.5)) * EoM.pulse(t, 2.5 + 0.75 / 0.7, 2.5 + 1 / 0.7))

    # define input function to be steer
    # put it in the form to also accept x (i.e., u=f(x,t))) but then ignore x
    # and return a vector even though it is length of one
    u_vec(_, t) = [steer(t)]

    # define time interval
    t1 = 0
    t2 = 20
    yoft = ltisim(result, u_vec, (t1, t2))

    # notation conflict, y is system output vector, but also lateral displacement
    # sensors are, in order, r, β, α_u, a_lat, y, θ, α_f, α_r

    # plot yaw rate vs time
    yidx = [1]
    p1 = ltiplot(system, yoft; yidx)

    # plot body slip angle vs time
    yidx = [2]
    p2 = ltiplot(system, yoft; yidx)

    # plot slip angles, understeer angle vs time
    yidx = [7, 8, 3]
    p3 = ltiplot(system, yoft; yidx)

    # plot lateral acceleration vs time
    yidx = [4]
    p4 = ltiplot(system, yoft; yidx)

    # plot path, noting that it is not even close to uniform scaling, x ~ 400 m, y ~ 2.5 m
    # becasue this plot is not a function of time, we need to use the EoM.plot function
    xlabel = "x [m]"
    ylabel = "y [m]"
    label = ""
    lw = 2 # thicker line weight
    size = (800, 400)
    p5 = EoM.plot(u * yoft.t, yoft[5, :]; xlabel, ylabel, label, lw, size)

    plots = [p1, p2, p3, p4, p5]

    # write all the results; steady state plots of outputs 1 through 4, 7, 8 (5 and 6 don't reach steady state)
    ss = :skip
    impulse = :skip
    summarize(system, result; plots, ss, impulse, format)

    #using EoM_X3D
    #animate_modes(system, result)

    # generate over a huge range of speeds to find characteristic speed
    vpts = 0.4:0.4:140


    system = [input_ex_yaw_plane(; u=x, m, a, b, Iz, cf, cr, ptf, ptr) for x in vpts]
    output = run_eom!.(system)
    result = analyze.(output; freq=(-1, 1))

    ss_resp = hcat(getproperty.(result, :ss_resp)...)
    if maximum(ss_resp[3, :]) > 0.5
        yy = LinearInterpolation(ss_resp[3, :], vpts)
        uchar = yy(0.5)
        println("Characteristic speed $(my_round(uchar)) m/s.")
        K = dpr * (a + b) * 9.81 / uchar^2
        println("Understeer gradient $(my_round(K)) degrees/g.")
    end

    println("Done.")

end

main()
