using EoM, ForwardDiff
include(joinpath("models", "input_ex_yaw_plane.jl"))

function main()

    # here you can enter your vehicle specs by name, including m, I, a, b, cf, cr; make sure you add the property you want to set to the argument list of input_ex_yaw_plane below after you set it; properties you don't set will use defaults defined in `input_ex_yaw_plane()`

    m = 1500
    a = 1.5
    b = 1.6

    # define the system
    u = 30
    system = input_ex_yaw_plane(; u, m, a, b)

    # generate the equations of motion
    output = run_eom!(system, true)

    # do the eigenvalues, freq resp
    result = analyze(output, true)

    # now lets try some closed loop feedback, where the driver input depends on the location; define the road geometry; note that we assume small heading angles in the linear equations, so can't do circuits yet

    function track(x)

        # define road y coordinate using the built-in function pulse to paste together a piecewise function of sines and constant

        y(x) = EoM.pulse(x, 50, 100) * (2 - 2 * cos(2π / 100 * (x - 50))) + EoM.pulse(x, 100, 150) * 4 + EoM.pulse(x, 150, 200) * (2 + 2 * cos(2π / 100 * (x - 150)))

        # use automatic differentiation to find the heading angle and curvature; as long as the angles are small we can approximate slope with the derivative and the curvature as the second derivative; automatic differentiation is a powerful numerical (i.e. not symbolic!) technique to compute the derivative of any function, using the fact that every function must be computed using basic arithmetic operations

        dy(x) = ForwardDiff.derivative(y, x)
        d2y(x) = ForwardDiff.derivative(dy, x)

        # evaluate all three and return those values
        y(x), dy(x), d2y(x)
    end

    # now let's define the driver model, based on the vehicle location and heading, and the road

    function steer_driver(y, t)

        # get vehicle location and heading from sensors (y is the output vector)
        offset = y[5]
        heading = y[6] * π / 180 # convert back to radians

        # get the road location, heading
        offset_t, heading_t, curvature = track(u * t)

        # find the error in location and heading
        offset_error = offset_t - offset
        heading_error = heading_t - heading

        # compute the appropriate steer angle to return to the road
        180 / π * ((a + b) * curvature + 1.1 * heading_error + 0.1 * offset_error)
    end

    # define a dummy function to convert the driver model from a function of the output to a function of the state, because the solver requires the input to be a function of the state
    steer(x, t) = [steer_driver(result.ss_eqns.C * x, t)]

    # define time interval
    t1 = 0
    t2 = 20
    # solve the equation of motion with the closed loop driver model
    yoft = ltisim(result, steer, (t1, t2))

    # notation conflict, y is system output vector, but also lateral displacement
    # sensors are, in order, r, β, α_u, a_lat, y, θ, α_f, α_r

    # plot yaw rate vs time
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
