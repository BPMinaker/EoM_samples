using EoM, ForwardDiff
# using EoM_X3D
include(joinpath("models", "input_ex_roll_centre.jl"))
# note that this mmodel has four actuators, one for each tire lateral force, so an external calculation is used for the tire model, anything we like, e.g. a magic formula
# the tire model is defined in the function tire(Z, slip), where Z is the vertical load and α is the slip angle

function main()

    # set the list of parameters that are not using defaults
    # here some of the values are set, but we can add many more if we like
    # see the input file for the list of parameters and their default values

    m = 1500
    u = 80 / 3.6
    a = 1.5
    b = 1.5
    cfy = 0
    cry = 0
    hf = 0.3
    hr = 0.4
    kf = 30000
    kr = 30000
    krf = 1500
    krr = 500

    format = :screen
    # format = :html

    # build system description with no cornering stiffnesses because will use a nonlinear tire model
    system = input_full_car_rc(; m, u, a, b, cfy, cry, hf, hr, kf, kr, krf, krr) # make sure to include all parameters here, and again below!!!
    output = run_eom!(system, true)
    result = analyze(output, true)

    # magic formula tire model parameters
    mtm = [1.6929, -55.2084E-6, 1.27128, 1601.8 * 180 / π, 6494.6, 4.7966E-3 * 180 / π, -0.3875E-3, 1.0]

    # define a nonlinear tire with load sensitivity
    function tire(Z, slip)
        C = mtm[1]
        D = (mtm[2] * Z .+ mtm[3]) .* Z
        B = mtm[4] * sin.(2 * atan.(Z / mtm[5])) / C ./ D
        Bslip = B .* slip
        E = mtm[7] * Z .+ mtm[8]

        -D .* sin.(C * atan.((1.0 .- E) .* Bslip + E .* atan.(Bslip)))
    end

    # get static tire normal loads (kN)
    # assume LF, LR, RF, RR sequence
    # display(getfield.(system.flex_points,:name))
    Z0 = vcat(getfield.(system.flex_points[[1, 2, 5, 6]], :preload)...)

    function track(x)
        # define road y coordinate using the EoM function `pulse()` to paste together a piecewise function of sines and constant; pulse(x, a, b) = 1 if a<x<b, 0 otherwise
        # this defines a value of y=0 for x<50, y ramps from 0 to 4 along a cosine curve for 50<x<100, y=4 for 100<x<150, y ramps from 4 back to 0 along a cosine for 150<x<200 and y=0 for x>200
        y(x) = EoM.pulse(x, 50, 100) * (2 - 2 * cos(2π / 100 * (x - 50))) + EoM.pulse(x, 100, 150) * 4 + EoM.pulse(x, 150, 200) * (2 + 2 * cos(2π / 100 * (x - 150)))

        # use automatic differentiation to find the heading angle and curvature; as long as the angles are small we can approximate heading with the slope found with the derivative and the curvature as the second derivative; automatic differentiation is a powerful numerical (i.e., not symbolic!) technique to compute the derivative of any function, using the fact that every function must be computed numerically using basic arithmetic operations (+-*/), and that the derivative of these operations is well defined
        dy(x) = ForwardDiff.derivative(y, x)
        d2y(x) = ForwardDiff.derivative(dy, x)

        # evaluate all three and return those values
        y(x), dy(x), d2y(x)
    end

    # now define the driver model, based on the vehicle location and heading, and the road
    function steer(y, t)
        # get vehicle location and heading from sensors (y is the output vector)
        offset = y[20]
        heading = y[21] * π / 180 # convert back to radians

        # get the road location, heading
        offset_t, heading_t, curvature = track(u * t)

        # find the error in location and heading
        offset_error = offset_t - offset
        heading_error = heading_t - heading

        # compute the appropriate steer angle to return to the road; note curvature is the inverse of the radius of curvature, so we it can be used to compute the kinematic steer angle, which is then modified based on the heading and offset errors
        (a + b) * curvature + 1.1 * heading_error + 0.1 * offset_error
    end

    # compute applied tire force
    function u_vec(x, t)
        # get sensor outputs (D=0)
        y = result.ss_eqns.C * x
        # get total normal load
        Z = Z0 - y[[3, 4, 9, 10]]
        # get slip angles from sensors, subtract steer on front
        δ = steer(y, t)
        α = y[[5, 6, 11, 12]] - [δ, 0, δ, 0]
        # compute tire force
        tire(Z, α)
    end

    println("Solving time history...")
    t1 = 0
    t2 = 15
    yoft = ltisim(result, u_vec, (t1, t2))

    # go back and recompute what the steer angle was, which is the output of the driver model (it wasn't recorded during the simulation because it is not an input or output of the system, the input is the tire force)
    delta = 180 / π * steer.(yoft.y, yoft.t)

    println("Plotting results...")
    # empty plot vector to push plots into
    plots = []

    # yaw rate
    yidx = [13]
    uidx = [0]
    label = ["Steer angle δ"]
    ylabel = ", δ [°]"
    p = ltiplot(system, yoft, delta; ylabel, label, yidx, uidx)
    push!(plots, p)


    # roll angle, pitch angle, slip angle, understeer angle
    yidx = [15, 16, 17, 21]
    label = ["Understeer angle α_u" "Steer angle δ"]
    ylabel = "Angles [°]"
    p = ltiplot(system, yoft, [yoft[18, :] .+ delta delta]; ylabel, label, yidx, uidx)
    push!(plots, p)

    # G lift
    yidx = [14]
    label = ["Steer angle δ"]
    ylabel = ", δ [°]"
    p = ltiplot(system, yoft, delta; ylabel, label, yidx, uidx)
    push!(plots, p)

    # lateral forces
    yidx = [0]
    uidx = [1, 2, 3, 4]
    ylabel = "Lateral forces [N]"
    p = ltiplot(system, yoft; ylabel, yidx, uidx)
    push!(plots, p)

    # yaw moment
    N = sum(yoft[[1, 2, 7, 8], :]; dims=1)[1, :]
    yidx = [1, 2, 7, 8]
    uidx = [0]
    label = ["Total"]
    ylabel = "Yaw moments [Nm]"
    p = ltiplot(system, yoft, N; ylabel, label, yidx, uidx, formatter=:plain)
    push!(plots, p)

    # plots not directly from inputs or outputs
    uidx = [0]
    yidx = [0]

    # get tire vertical forces
    ZZ = Z0' .- yoft[[3, 4, 9, 10], :]'

    label = ["Tire vertical force Z_lf" "Tire vertical force Z_lr" "Tire vertical force Z_rf" "Tire vertical force Z_rr"]
    ylabel = "Vertical forces [N]"
    p = ltiplot(system, yoft, ZZ; ylabel, label, yidx, uidx)
    push!(plots, p)

   # find weight transfer
    ΔZ = 0.5 * [ZZ[:, 3] - ZZ[:, 1] ZZ[:, 4] - ZZ[:, 2]]

    label = ["Front weight transfer" "Rear weight transfer"]
    ylabel = "Lateral weight transfer [N]"
    p = ltiplot(system, yoft, ΔZ; ylabel, label, yidx, uidx)
    push!(plots, p)

    # get tire slip angles
    slips = 180 / π * yoft[[5, 6, 11, 12], :]' - delta .* [1, 0, 1, 0]'

    label = ["Tire slip angle α_lf" "Tire slip angle α_lr" "Tire slip angle α_rf" "Tire slip angle α_rr"]
    ylabel = "Slip angles [°]"
    p = ltiplot(system, yoft, slips; ylabel, label, yidx, uidx)
    push!(plots, p)

    # get tire lateral forces
    YY = hcat(yoft.u...)

    # find total front and rear tire forces
    YF = (YY[1, :] + YY[3, :])
    YR = (YY[2, :] + YY[4, :])

    # get tire forces using slip but with static normal loads
    Y0 = hcat([tire.(Z0, i * π / 180) for i in eachrow(slips)]...)

    # find total front and rear tire forces
    YF0 = (Y0[1, :] + Y0[3, :])
    YR0 = (Y0[2, :] + Y0[4, :])

    # get the difference between the static max and actual tire forces
    ΔYF = YF0 - YF
    ΔYR = YR0 - YR

    label = ["Front grip loss" "Rear grip loss"]
    ylabel = "Lateral grip loss due to weight transfer [N]"
    p = ltiplot(system, yoft, [ΔYF ΔYR]; ylabel, label, yidx, uidx)
    push!(plots, p)

    acc = sum(YY, dims=1)[1, :] * 9.81 / sum(Z0)
    label = ["ru" "Σf/m" "vdot"]
    ylabel = "Lateral accel'n [m/s^2]"
    p = ltiplot(system, yoft, [yoft[19, :] acc acc - yoft[19, :]]; ylabel, label, yidx, uidx)
    push!(plots, p)

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
    p = EoM.plot(x, [yoft[20, :] path]; xlabel, ylabel, label, lw, size)
    push!(plots, p)

    println("Plotted results.")

    # compute cornering stiffnesses from the magic tire model
    cfy = mtm[4] * sin.(2 * atan.(Z0[1] / mtm[5]))
    cry = mtm[4] * sin.(2 * atan.(Z0[2] / mtm[5]))

    # rebuild the equations of motion using the updated cornering stiffnesses
    # really only need to do this if we want to see the effect of the cornering stiffnesses on the yaw eigenvalues
    system = input_full_car_rc(; m, u, a, b, cfy, cry, hf, hr, kf, kr, krf, krr)
    output = run_eom!(system, false)
    result = analyze(output, false)

    bode = :skip
    impulse = :skip
    ss = :skip
    summarize(system, result; plots, bode, ss, impulse, format)

    # generate animations of the mode shapes
    # animate_modes(system, result, true)

    println("Done.")

end

@time main()
