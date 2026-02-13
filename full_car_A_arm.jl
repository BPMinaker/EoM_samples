using EoM
using EoM_X3D
using Interpolations
using Dates

include(joinpath("models", "input_ex_full_car_A_arm.jl"))

# format = :screen
format = :html

r = 0.315
u = 22.4 # forward speed in m/s (50 mph)
t1 = 0
t2 = 10

function main()
    dtr = π / 180
    rpm = 6 * dtr

    # set the parameters that can't be adjusted by the student
    m = 1565
    a = 2.631 * (1 - fwf)
    b = 2.631 * fwf
    tf =1.5
    tr = 1.5
    hG = 0.5
    Ix = 818 # moments of inertia
    Iy = 3267
    Iz = 3508
    muf = 50 # unsprung mass, front
    mur = 50
    kt = 180000 # tire vertical stiffness

    # because we are using a nonlinear model of the tire, we have to set the cornering stiffness of the linear tire model to zero
    cfy = 0
    cry = 0

    params = list(; u, m, a, b, tf, tr, hG, Ix, Iy, Iz, kf, kr, cf, cr, krf, krr, muf, mur, cfy, cry, kt, gear, fd)

    # build system description with no cornering stiffnesses because will use a nonlinear tire model
    system = input_full_car_a_arm(;params, front, rear) # make sure to include all parameters you want to change here
    sensors_animate!(system)
    output = run_eom!(system, true)
    result = analyze(output, true; bode=:skip, impulse=:skip, ss=:skip)

    # get static tire normal loads (kN)
    Z0 = [system.flex_points_name[i].preload[1] for i in ["LF tire, Z", "LR tire, Z", "RF tire, Z", "RR tire, Z"]]

    # find the output indices for tire normal loads and slip angles
    Zidx = [system.sidx[i] for i in ["Z_lf", "Z_lr", "Z_rf", "Z_rr"]]
    αidx = [system.sidx[i] for i in ["α_lf", "α_lr", "α_rf", "α_rr"]]

    # find the input indices for tire lateral loads and bumps
    Yidx = [system.aidx[i] for i in ["Y_lf", "Y_lr", "Y_rf", "Y_rr"]]
    uuidx = [system.aidx[i] for i in ["u_lf", "u_lr", "u_rf", "u_rr"]]

    # compute applied tire force
    function u_vec1(x, t, mode, gain=1.0)
        # get sensor outputs (D=0 for these rows)
        y = result.ss_eqns.C * x
        # get total normal load
        Z = Z0 - y[Zidx]
        # get slip angles from sensors, subtract steer on front, correct for units
        if mode == :sweep
            str = t * dtr
        elseif mode == :dlc
            str = gain * steer(t) * dtr
        end
        α = y[αidx] - [str, 0, str, 0]
        α .*= [1, 1, -1, -1] # flip sign on RF and RR slip angles (modified iso sign convention)

        uu = zeros(length(system.actuators))

        # compute tire force, ignore camber effect, restoring moment
        uu[Yidx] = tire(Z, α, [0, 0, 0, 0])[1]
        uu[Yidx] .*= [-1, -1, 1, 1] # flip sign on LF and LR tire forces (mirror)

        uu
    end

    println("Solving time history for steer sweep...")
    ptr1(x, t) = u_vec1(x, t, :sweep)
    yoft_yaw_sweep = ltisim(result, ptr1, (t1, t2 / 2))

    # get lateral force inputs as a matrix
    Y = hcat(yoft_yaw_sweep.u...)[Yidx, :]
    # sum columns to get total lateral force and divide by total weight to get lateral acceleration in g
    acc = sum(Y, dims=1)[1, :] / sum(Z0)

    #    display(ltiplot(system, yoft_yaw_sweep, [acc yoft_yaw_sweep.t]; ylabel="Lateral accel'n [g], δ [°]", label=["Lat acc" "Steer angle δ"], yidx=[0], uidx=[0]))

    interp = LinearInterpolation(acc, yoft_yaw_sweep.t)
    gain = interp(0.3)
    println("Steer gain for 0.3 m/s^2 lateral acceleration: ", round(gain, digits = 3))

    mult = 3.0
    ptr2(x, t) = u_vec1(x, t, :dlc, mult * gain)

    println("Solving time history for double lane change...")
    pass = true
    while pass
        println("Testing gain: ", mult)
        yoft_yaw_test = ltisim(result, ptr2, (t1, t2))

        # δ = mult * gain * steer.(yoft_yaw_test.t)
        # display(ltiplot(system, yoft_yaw_test, δ; ylabel=", δ [°]", label=["Steer angle δ"], sidx=["r"], uidx=[0]))

        idx1 = findfirst(yoft_yaw_test.t .== 3.07) # 2 + 1.07
        println("Checking displacement at t = 3.07 s: ", round(yoft_yaw_test[system.sidx["y"], :][idx1], digits=3), " m")
        cond1 = yoft_yaw_test[system.sidx["y"], :][idx1] > 1.83

        max_yaw = minimum(yoft_yaw_test[system.sidx["r"], :])
        println("Maximum yaw rate during DLC: ", round(max_yaw, digits=3), " deg/s")

        idx2 = findfirst(yoft_yaw_test.t .== 4.929) # 2.5 + 1 / 0.7 + 1
        println("Checking yaw rate at t = 4.929 s: ", round(yoft_yaw_test[system.sidx["r"], :][idx2], digits=3), " deg/s")
        cond2 = yoft_yaw_test[system.sidx["r"], :][idx2] > max_yaw * 0.35

        idx3 = findfirst(yoft_yaw_test.t .== 5.679) # 2.5 + 1 / 0.7 + 1.75
        println("Checking yaw rate at t = 5.679 s: ", round(yoft_yaw_test[system.sidx["r"], :][idx3], digits=3), " deg/s")
        cond3 = yoft_yaw_test[system.sidx["r"], :][idx3] > max_yaw * 0.20

        println("Conditions: ", cond1, ", ", cond2, ", ", cond3)

        if cond1 && (cond2 || cond3)
            mult += 0.25
        else
            pass = false
            mult -= 0.25
            println("Final steer gain factor: ", mult)
        end
    end

    yoft_yaw = ltisim(result, ptr2, (t1, t2))
    animate_history(system, yoft_yaw)

    function u_vec2(_, t)
        uu = zeros(length(system.actuators))

        uu[uuidx] = [zofxl(u * t),
            zofxl(u * t - a - b),
            zofxr(u * t),
            zofxr(u * t - a - b)]
        uu
    end

    println("Solving time history for ride quality...")
    yoft_bounce = ltisim(result, u_vec2, (t1, t2))

    # because our input vector must be a function, we interpolate the output of the previous simulation to get the vertical acceleration at the passenger location, not actually an interpolation, but makes the acceleration a function of time
    interp_z = LinearInterpolation(yoft_bounce.t, yoft_bounce[system.sidx["zddot_P"], :])
    # use a dummy function to ignore the first argument and return the result as a vector
    dummy(_, t) = [interp_z(t)]
    filter = ride_filter()
    println("Applying ride filter to passenger vertical acceleration...")
    yoft_filt = ltisim(filter, dummy, (t1, t2))

    # compute RMS vertical acceleration at passenger location
    rms_pass_acc = sqrt((sum(yoft_filt[1, :]) .^ 2) / length(yoft_filt.t))
    println("Filtered passenger RMS vertical acceleration: ", round(rms_pass_acc, digits=5), " m/s^2")
    animate_history(system, yoft_bounce)

    η = 0.95 # driveline efficiency
    μ = 0.9 # friction limit
    # assume 5% tire slip, for purpose of computing shift speeds
    slip = 1.05

    ti = [30, 50, 75, 115, 150, 160, 150, 120]
    ωi = [0, 1000, 2000, 3000, 4000, 5000, 6000, 7000]
    te = LinearInterpolation(ωi, ti)

    # calculate shift speeds (at redline)
    redline = ωi[end]
    vmax = 3.6 * redline * rpm * r / slip / fd ./ gear
    println("Shift speeds [km/h]:")
    display(round.(vmax, digits=2))

    params.u = 0.01
    system = input_full_car_a_arm(;params, front, rear) # make sure to include all parameters you want to change here
    sensors_animate!(system)
    output = run_eom!(system, true)
    result = analyze(output, true; bode=:skip, impulse=:skip, ss=:skip)

    xGidx = system.sidx["x_G"]
    uGidx = system.sidx["u_G"]

    midx = [system.aidx[i] for i in ["m_lr", "m_rr"]]
    Xaidx = system.aidx["Xa"]

    function u_vec3(x, t)

        uu = zeros(length(system.actuators))
        y = result.ss_eqns.C * x

        ρ = 1.23
        af = 2.85
        cd = 0.35
        uu[Xaidx] = ρ / 2 * af * cd * y[uGidx] * abs(y[uGidx])

        n = findnext(y[uGidx] .< vmax, 1)
        if isnothing(n) # if we are above redline in top gear
            n = length(gear) # use top gear
        end

        ω = y[uGidx] * gear[n] * fd * slip / rpm / r
        ω < 0 && (ω = 0)
        ω > redline && (ω = redline)
        m = te(ω) * gear[n] * fd * η

        # find maximum traction force limited torque
        Z = Z0 - y[Zidx]
        mmax = μ * 0.5 * (Z[1] + Z[2]) * r
        if m > mmax
            m = mmax
        end

        # include rolling resistance loss
        mm = m - (0.013 + 6.5e-6 * y[uGidx]^2) * 9.81 * m * sign(y[uGidx]) * r
        uu[midx] = [mm, mm]

        uu
    end

    println("Solving time history for longitudinal motion...")
    yoft_long = ltisim(result, u_vec3, (t1, 3 * t2))
    animate_history(system, yoft_long)

    interp_xt = LinearInterpolation(yoft_long[xGidx, :], yoft_long.t)
    interp_tu = LinearInterpolation(yoft_long.t, yoft_long[uGidx, :])

    if yoft_long[xGidx, :][end] > 402.336
        tq = interp_xt(402.336)
        println("Quarter mile time: ", round(tq, digits=2), " s at ", round(interp_tu(tq), digits=2), " km/h.")
    else
        println("Simulation ended before 1/4 mile!  Increase the time interval.")
    end

    println("Plotting results...")
    # empty plot vector to push plots into
    plots = []

    δ = mult * gain * steer.(yoft_yaw.t)

    # yaw rate
    sidx = ["r"]
    uidx = [0]
    label = ["Steer angle δ"]
    ylabel = ", δ [°]"
    p = ltiplot(system, yoft_yaw, δ; ylabel, label, sidx, uidx)
    push!(plots, p)

    # roll angle, pitch angle, slip angle, understeer angle
    sidx = ["ϕ", "θ", "β"]
    label = ["Understeer angle α_u" "Steer angle δ"]
    ylabel = "Angles [°]"
    p = ltiplot(system, yoft_yaw, [yoft_yaw[system.sidx["α_u-δ"], :] .+ δ δ]; ylabel, label, sidx, uidx)
    push!(plots, p)

    # G lift
    sidx = ["z_G"]
    label = ["Steer angle δ"]
    ylabel = ", δ [°]"
    p = ltiplot(system, yoft_yaw, δ; ylabel, label, sidx, uidx)
    push!(plots, p)

    # lateral forces
    yidx = [0]
    uidx = Yidx
    ylabel = "Lateral forces [N]"
    p = ltiplot(system, yoft_yaw; ylabel, yidx, uidx)
    push!(plots, p)

    # plots not directly from inputs or outputs
    uidx = [0]
    yidx = [0]

    # get tire vertical forces
    Z = Z0' .- yoft_yaw[Zidx, :]'

    label = ["Tire vertical force Z_lf" "Tire vertical force Z_lr" "Tire vertical force Z_rf" "Tire vertical force Z_rr"]
    ylabel = "Vertical forces [N]"
    p = ltiplot(system, yoft_yaw, Z; ylabel, label, yidx, uidx)
    push!(plots, p)

    # find weight transfer
    ΔZ = 0.5 * [Z[:, 3] - Z[:, 1] Z[:, 4] - Z[:, 2]]

    label = ["Front weight transfer" "Rear weight transfer"]
    ylabel = "Lateral weight transfer [N]"
    p = ltiplot(system, yoft_yaw, ΔZ; ylabel, label, yidx, uidx)
    push!(plots, p)

    # get tire slip angles
    α = 180 / π * yoft_yaw[αidx, :]'
    α[:, [1, 3]] .-= δ
    label = ["Tire slip angle α_lf" "Tire slip angle α_lr" "Tire slip angle α_rf" "Tire slip angle α_rr"]
    ylabel = "Slip angles [°]"
    p = ltiplot(system, yoft_yaw, α; ylabel, label, yidx, uidx)
    push!(plots, p)

    # get tire lateral forces
    YY = hcat(yoft_yaw.u...)[Yidx, :]
    acc = sum(YY, dims=1)[1, :] / sum(Z0)
    label = ["ΣY/m"]
    ylabel = "Lateral accel'n [g]"
    p = ltiplot(system, yoft_yaw, acc; ylabel, label, yidx, uidx)
    push!(plots, p)

    ###############################################

    aidx = ["u_lf", "u_rf"]
    p = ltiplot(system, yoft_bounce; yidx, aidx)
    push!(plots, p)

    # roll angle, pitch angle
    sidx = ["ϕ", "θ"]
    ylabel = "Angles [°]"
    p = ltiplot(system, yoft_bounce; ylabel, label, sidx, uidx)
    push!(plots, p)

    # G lift
    sidx = ["z_G"]
    p = ltiplot(system, yoft_bounce; sidx, uidx)
    push!(plots, p)

    # P lift
    sidx = ["zddot_P"]
    p = ltiplot(system, yoft_bounce; sidx, uidx)
    push!(plots, p)

    # get tire vertical forces
    Z = Z0' .- yoft_bounce[Zidx, :]'

    label = ["Tire vertical force Z_lf" "Tire vertical force Z_lr" "Tire vertical force Z_rf" "Tire vertical force Z_rr"]
    ylabel = "Vertical forces [N]"
    p = ltiplot(system, yoft_bounce, Z; ylabel, label, yidx, uidx)
    push!(plots, p)

    ###############################################

    # plot speed
    sidx = ["u_G"]
    p = ltiplot(system, yoft_long; sidx, uidx)
    push!(plots, p)


    # plot axle torque
    yidx = [0]
    aidx = ["m_lr", "m_rr"]
    p = ltiplot(system, yoft_long; yidx, aidx)
    push!(plots, p)

    # pitch angle
    sidx = ["θ"]
    ylabel = "Pitch angle [°]"
    p = ltiplot(system, yoft_long; ylabel, label, sidx, uidx)
    push!(plots, p)

    # G lift
    sidx = ["z_G"]
    p = ltiplot(system, yoft_long; sidx, uidx)
    push!(plots, p)

    # get tire vertical forces
    Z = Z0' .- yoft_long[Zidx, :]'

    label = ["Tire vertical force Z_lf" "Tire vertical force Z_lr" "Tire vertical force Z_rf" "Tire vertical force Z_rr"]
    ylabel = "Vertical forces [N]"
    p = ltiplot(system, yoft_long, Z; ylabel, label, yidx, uidx)
    push!(plots, p)

    println("Plotted results.")
    summarize(system, result; plots, format)

    println("Writing $team_name's result...")
    open(joinpath("output", "output.txt"), "a") do io
        println(io, "Multiplier, $mult, Ride RMS vertical acceleration, $rms_pass_acc, Drag strip time, $tq")
    end

    # generate animations of the mode shapes
    # animate_modes(system, result, true)

end

println("Starting...")

# define a steer function
steer(t) = sin(2π * 0.7 * (t - 2)) * EoM.pulse(t, 2, 2 + 0.75 / 0.7) - EoM.pulse(t, 2 + 0.75 / 0.7, 2.5 + 0.75 / 0.7) + sin(2π * 0.7 * (t - 2.5)) * EoM.pulse(t, 2.5 + 0.75 / 0.7, 2.5 + 1 / 0.7)

# define random road profile functions
zofxl, zofxr = random_road(class=5, dz=0.2, L=(t2 - t1) * u)

flist = readdir("specifications"; join = true)
time = Dates.format(now(), "HH:MM:SS")
!isdir("output") && (mkpath("output"))
open(joinpath("output", "output.txt"), "w") do io
    println(io, "This is the output recorded at $time.")
end
for file in flist
    println("Including file: ", file)
    include(file)
    println("Running $team_name's simulation...")
    open(joinpath("output", "output.txt"), "a") do io
        print(io, "$team_name, ")
    end
    main()
end

println("Done.")
