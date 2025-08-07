using EoM
include(joinpath("models", "input_ex_yaw_plane.jl"))
include(joinpath("models", "track.jl"))
include(joinpath("models", "driver.jl"))

function main()

    # here you can enter your vehicle specs by name, including m, I, a, b, cf, cr; make sure you add the property you want to set to the argument list of input_ex_yaw_plane below after you set it; properties you don't set will use defaults defined in `input_ex_yaw_plane()`

    m = 1500
    a = 1.5
    b = 1.6
    l = a + b

    # define the system
    u = 30
    system = input_ex_yaw_plane(; u, m, a, b)

    # generate the equations of motion
    output = run_eom!(system, true)

    # do the eigenvalues, freq resp
    result = analyze(output, true)

    # now lets try some closed loop feedback, where the driver input depends on the location

    # define a dummy function to convert the driver model from a function of the output to a function of the state, because the solver requires the input to be a function of the state
    function u_vec(x, t)
        y = result.ss_eqns.C * x
        # get vehicle location and heading from sensors (y is the output vector)
        offset = y[system.sidx["y"]]
        heading = y[system.sidx["ψ"]] * π / 180 # convert back to radians
        [180/π * driver(l, offset, heading, u * t)]
    end

    # define time interval
    t1 = 0
    t2 = 20
    # solve the equation of motion with the closed loop driver model
    yoft = ltisim(result, u_vec, (t1, t2))

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
    sidx = ["a_lat"]
    p4 = ltiplot(system, yoft; sidx)

    # plot path, noting that it is not even close to uniform scaling, x ~ 400 m, y ~ 2.5 m
    # becasue this plot is not a function of time, we need to use the EoM.plot function
    xlabel = "x [m]"
    ylabel = "y [m]"
    label = ["Path" "Target path"]
    lw = 2 # thicker line weight
    size = (800, 400)

    x = u * yoft.t
    track_y(x) = track(x)[1]
    path = track_y.(x)
    p5 = EoM.plot(x, [yoft[5, :] path]; xlabel, ylabel, label, lw, size)

    plots = [p1, p2, p3, p4, p5]

    # write all the results; steady state plots of outputs 1 through 4, 7, 8 (5 and 6 don't reach steady state)
    ss = [1, 1, 1, 1, 0, 0, 1, 1]
    impulse = :skip
    bode = :skip
    summarize(system, result; plots, ss, impulse, bode)

    println("Done.")

end

main()
