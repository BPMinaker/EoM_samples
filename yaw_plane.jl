using EoM
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
    cf = 1437 # front tire cornering stiffness in N/°
    cr = 1507 # rear tire cornering stiffness in N/°
    df = 34 # front tire self-aligning moment stiffness in Nm/°
    dr = 38 # rear tire self-aligning moment stiffness in Nm/°

    ptf = df / cf # front pneumatic trail 
    ptr = dr / cr # rear pneumatic trail
    # we adjust the location of the front and rear tires (modelled as flex_points) to account for the pneumatic trail, which is a function of the self-aligning moment stiffness and the cornering stiffness
    # the pneumatic trail is small compared to the wheelbase, but it effectively moves the centre of mass forward relative to the tires, so it has an understeering effect that can be significant

    cf *= 2 * dpr # front axle cornering stiffness in N/rad
    cr *= 2 * dpr # rear axle cornering stiffness in N/rad
    df *= 2 * dpr # front axle self-aligning moment stiffness in Nm/rad
    dr *= 2 * dpr # rear axle self-aligning moment stiffness in Nm/rad

    # generate our vector of systems
    system = [input_ex_yaw_plane(; u=x, m, a, b, Iz, cf, cr, ptf, ptr) for x in vpts]

    # generate the equations of motion, but many times, for every different value of forward speed
    output = run_eom!.(system)

    # do the eigenvalues, freq resp, etc, for each forward speed, but skip the impulse response because it is not very informative for this system, and it takes a long time to compute
    result = analyze.(output; freq=(-1, 1), impulse = :skip)

    # write all the results; steady state plots of outputs 1 through 4, 7, 8 (5 and 6 don't reach steady state)
    summarize(vpts, result; format)

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
    t2 = 8
    yoft = ltisim(result, u_vec, (t1, t2))
    # notation conflict, y is system output vector, but also lateral displacement
    # so call the output vector yoft, for y of t, to avoid confusion

    # generate plots of the time history
    println("Plotting results...")
    plots = [ltiplot(yoft; sidx = i) for i in [["r"], ["β"], ["α_f", "α_r", "α_u"], ["a_y"]]]

    # plot path, noting that it is not even close to uniform scaling, x ~ 200 m, y ~ 3 m
    # becasue this plot is not a function of time, we need to use the plot function
    xlabel = "x [m]"
    ylabel = "y [m]"
    label = ""
    yidx= system.sidx["y"]
    p5 = plot(u * yoft.t, yoft[yidx, :]; xlabel, ylabel, label)
    push!(plots, p5)

    # write all the results; steady state plots of outputs (except y and ψ, which don't reach steady state), eignenvalues, bode plots, time history plots
    summarize(result; plots, format)

    #using EoM_X3D
    #animate_modes(system, result)
end

println("Starting...")
main()
println("Done.")
