using EoM, Interpolations
# read in the function that will define our drag race vehicle model
include(joinpath("models", "input_ex_drag_race.jl"))

function main()

    format = :screen
    # format = :html

    # set the values of the parameters here
    # tire radius in meters, for example, for 195/50-R16
    re = (8 * 25.4 + 0.5 * 195) / 1000
    η = 0.9 # driveline efficiency
    gear = [6, 5, 4, 3, 2, 1] # ratios, starting from 1st
    fd = 4.1 # final drive ratio
    a = 1.2 # cg location [m]
    b = 1.4

    m = 1600 # mass
    h_G = 0.4 # height of centre of mass [m]
    μ = 0.9 # friction limit
    # assume 5% tire slip, for purpose of computing shift speeds
    slip = 1.05
    rdf = 1.0 # rear drive fraction

    # input engine specs
    # engine torque [Nm] in evenly spaced engine speed increments from 0 to redline
    # we need to fudge a little and have a torque value even at zero speed, otherwise no launch
    ti = [30, 50, 75, 115, 150, 160, 150, 120]
    ωi = [0, 1000, 2000, 3000, 4000, 5000, 6000, 7000]
    redline = ωi[end]

    # make the fit, linear is not the best, but good enough
    te = LinearInterpolation(ωi, ti)
    # te is now a callable function, i.e., te(ω) returns torque (in Nm) at engine speed ω (in rpms)

    # calculate shift speeds (at redline)
    vmax = redline * 2π / 60 * re / slip / fd ./ gear
    println("Shift speeds [km/h]:")
    display(round.(3.6 * vmax, digits=2))

    # suspension stiffness and damping - the default values should be close enough
    cf = 4000 # front suspension damping [Ns/m]
    cr = 4000 # rear damping
    kf = 30000 # front suspension stiffness [N/m]
    kr = 30000

    # build the equations of motion
    # define the function that builds our model, with our specs
    system = input_ex_drag_race(; re, a, b, m, h_G, kf, kr, cf, cr)

    # pass the name of the that function to run_eom, which will build the equations of motion
    output = run_eom!(system)

    # static preloads are computed as part of the generation of the equations of motion
    # we need these for the tire model, so save them in convenient notation
    Zf0 = system.flex_points_name["front spring"].preload[1]
    Zr0 = system.flex_points_name["rear spring"].preload[1]
    println("Static Zf=", round(Zf0, digits=2), " N")
    println("Static Zr=", round(Zr0, digits=2), " N")

    # do some analysis, to convert the equations to state space form
    result = analyze(output)

    # save the state space matrices in a convenient notation
    (; A, B, C, D) = result.ss_eqns

    uuidx = system.sidx["u_G"]

    Xaidx = system.aidx["X_a"]
    Xtidx = system.aidx["X_t"]

    Zsfidx = system.sidx["Zs_f"]
    Zsridx = system.sidx["Zs_r"]
    Zdfidx = system.sidx["Zd_f"]
    Zdridx = system.sidx["Zd_r"]

    # build the function that is the input to the equation of motion (i.e., the external forces)
    # we will call this function through the ODE solver
    function u_vec(x, t)
        # println("t ",t)
        # define the vector and set 0 values as placeholders
        # we will call this u instead of u to keep it distinct from the function name, even though Julia doesn't care
        u = [0.0; 0.0]
        # u[1] aero resistance force
        # u[2] traction force
        # note that these are defined in the input definition file
        # we will compute them and store them in u_vec

        # find the outputs of the model
        # these are also defined in the input definition file
        y = C * x
        # y[1] long position
        # y[2] long velocity
        # y[3] long accl'n (note that we we can't use this to find u because it depends directly on u! - the third row of D has non-zero entries)
        # y[4] rear axle normal spring force
        # y[5] rear axle damper force
        # y[6] front axle normal spring force
        # y[7] front axle damper force


        # calculate air resistance from outputs
        ρ = 1.23
        af = 2.85
        cd = 0.35
        u[Xaidx] = ρ / 2 * af * cd * y[uuidx] * abs(y[uuidx]) # use speed times abs(speed) instead of speed squared to switch force direction if we are moving backwards (which should never happen, but sometimes the rolling resistance model can cause weird things at very low speed)

        # find which gear we should use
        # note the .< returns a vector, i.e, (y[2] .< vmax) returns a vector of true or false where the current speed is below the shift point, we use findnext to find the index of the first true
        n = findnext(y[uuidx] .< vmax, 1)
        if isnothing(n) # if we are above redline in top gear
            n = length(gear) # use top gear
        end

        # find engine speed (in rpm)
        ω = y[uuidx] * gear[n] * fd * slip * 60 / 2π / re

        # if somehow we are moving backwards, set w=0, to prevent interpolation error (although it means we must have an issue somewhere else!)
        # note the shortcut conditional, in lieu of if statement
        ω < 0 && (ω = 0)

        # rev limiter, again shortcut conditional
        ω > redline && (ω = redline)

        # calculate traction force (axle torque/radius)
        X = te(ω) * gear[n] * fd * η / re
        Xr = rdf * X
        Xf = (1 - rdf) * X

        # calculate axle load, static plus dynamic load, note the dynamic is negative if the spring is in compression, but this should add to static, hence negative sign
        Zr = Zr0 - y[Zsridx] - y[Zdridx]
        Zf = Zf0 - y[Zsfidx] - y[Zdfidx]

        # check for wheelspin, conditional
        Xrmax = μ * Zr
        Xfmax = μ * Zf
        if Xr > Xrmax
            Xr = Xrmax
            Xfmax = min((1 - rdf) / rdf * Xr, Xfmax)
        end
        if Xf > Xfmax
            Xf = Xfmax
            Xrmax = min(rdf / (1 - rdf) * Xf, Xrmax)
        end
        if Xr > Xrmax
            Xr = Xrmax
        end
        if Xf > Xfmax
            Xf = Xfmax
        end

        # include rolling resistance loss, and note sign function to reverse force if needed, set traction force
        u[Xtidx] = Xr + Xf - (0.013 + 6.5e-6 * y[2]^2) * 9.81 * m * sign(y[2])

        # return u
        u
    end

    # solve the ODE
    # choose the time interval and step size
    println("Solving time history...")
    t1 = 0
    t2 = 30
    # pass the state space matrices, the input function, and the time interval to the solver
    # it will assume zeros as inital conditions
    yoft = ltisim(result, u_vec, (t1, t2))

    y = yoft[:, :]'
    # convert the vector of vectors to a matrix

    x = y[:, 1]
    uu = y[:, 2] * 3.6 # scale second column to convert to km/h
    aG = y[:, 3] / 9.81 # acc'n in g

    ZZr = y[:, 4] + y[:, 5]
    # add damping load to spring load
    ZZr *= -1.0 # take compressive loads as positve (switch signs)
    ZZr .+= Zr0 # add static load, note .+= increments each entry in the vector
    ZZr /= 1000 # scale third column to get kN
    ZZf = y[:, 6] + y[:, 7]
    ZZf *= -1.0 # take compressive loads as positve (switch signs)
    ZZf .+= Zf0 # add static load, note .+= increments each entry in the vector
    ZZf /= 1000 # scale third column to get kN

    # linear interpolation to find 1/4 mile, 60 mph times
    # note 1/4 mile = 402.336 m, 60 mph = 96.5606 km/h
    interp_xt = LinearInterpolation(x, yoft.t)
    interp_tu = LinearInterpolation(yoft.t, uu)
    interp_ut = LinearInterpolation(uu, yoft.t)

    # find quarter mile time and speed, round it, and print
    if x[end] > 402.336
        tf = interp_xt(402.336)
        println("Quarter mile time: ", round(tf, digits=2), " s at ", round(interp_tu(tf), digits=1), " km/h.")
    else
        println("Simulation ended before 1/4 mile!  Increase the time interval.")
    end

    # find 0-60 time, round it, and print
    if uu[end] > 96.5606
        println("0-60 mph time: ", round(interp_ut(96.5606), digits=2), " s.")
    else
        println("Simulation ended below 60 mph!")
    end

    # don't include any of the inputs in the plots
    uidx = [0]

    # make the first plot and save it in a vector
    # we can just choose output number one, which is the distance
    sidx = ["x_G"]
    plots = [ltiplot(system, yoft; sidx, uidx)]

    # make the next plot, and push it onto the plot vector
    # we can't just choose output number two, which is the velocity, because we want new units
    # manually set the label and ylabel
    yidx = [0]
    label = hcat("Velocity u")
    ylabel = "u [km/h]"
    push!(plots, ltiplot(system, yoft, uu; ylabel, label, yidx, uidx))

    label = hcat("Acceleration u_dot")
    ylabel = "u dot [g]"
    push!(plots, ltiplot(system, yoft, aG; ylabel, label, yidx, uidx, ylims=(0, Inf)))

    label = ["Z_r" "Z_f"]
    ylabel = "Axle vertical load [kN]"
    push!(plots, ltiplot(system, yoft, [ZZr ZZf]; ylabel, label, yidx, uidx, ylims=(0, Inf)))

    # pass all the results and plots, skip the Bode plots for now
    bode = :skip
    impulse = :skip
    summarize(system, result; plots, bode, impulse, format)

    println("Done.")

end

main()
