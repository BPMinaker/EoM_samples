using EoM
include(joinpath("models", "input_ex_roll_centre.jl"))
include(joinpath("models", "track.jl"))
include(joinpath("models", "driver.jl"))
# note that this mmodel has four actuators, one for each tire lateral force, so an external calculation is used for the tire model, anything we like, e.g. a magic formula
# the tire model is defined in the function tire(Z, α, γ), where Z is the vertical load, α is the slip angle, and γ is the camber angle

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
    r = 0.315

    l = a + b
    format = :screen
    # format = :html

    # build system description with no cornering stiffnesses because will use a nonlinear tire model
    system = input_full_car_rc(; m, u, a, b, cfy, cry, hf, hr, kf, kr, krf, krr, r) # make sure to include all parameters here, and again below!!!
    output = run_eom!(system, true)
    result = analyze(output, true)

    # get static tire normal loads (kN)
    Z0 = [
        system.flex_points_name["LF tire, Z"].preload[1],
        system.flex_points_name["LR tire, Z"].preload[1],
        system.flex_points_name["RF tire, Z"].preload[1],
        system.flex_points_name["RR tire, Z"].preload[1]
    ]

    # find the output indices for tire normal loads and slip angles
    Zidx = get.([system.sidx], ["Z_lf", "Z_lr", "Z_rf", "Z_rr"], 0)
    αidx = get.([system.sidx], ["α_lf", "α_lr", "α_rf", "α_rr"], 0)

    # compute applied tire force
    function u_vec(x, t)
        # get sensor outputs (D=0)
        y = result.ss_eqns.C * x
        # get total normal load
        Z = Z0 - y[Zidx]

        # get vehicle location and heading from sensors (y is the output vector)
        offset = y[system.sidx["y"]]
        heading = y[system.sidx["ψ"]] * π / 180 # convert back to radians
        δ = driver(l, offset, heading, u * t)

        # get slip angles from sensors, subtract steer on front
        # flip sign on RF and RR slip angles
        α = (y[αidx] - [δ, 0, δ, 0]) .* [1, 1, -1, -1]
        # compute tire force, ignore camber effect, restoring moment
        Y = tire(Z, α, [0, 0, 0, 0])[1]
        Y .* [-1, -1, 1, 1] # flip sign on LF and LR tire forces (mirror)
    end

    println("Solving time history...")
    t1 = 0
    t2 = 15
    yoft = ltisim(result, u_vec, (t1, t2))

    # go back and recompute what the steer angle was, which is the output of the driver model (it wasn't recorded during the simulation because it is not an input or output of the system, the input is the tire force)

    δ = 180 / π * driver.(l, yoft[system.sidx["y"], :], yoft[system.sidx["ψ"], :] * π / 180, u * yoft.t)

    println("Plotting results...")
    # empty plot vector to push plots into
    plots = []

    # yaw rate
    sidx = ["r"]
    uidx = [0]
    label = ["Steer angle δ"]
    ylabel = ", δ [°]"
    p = ltiplot(system, yoft, δ; ylabel, label, sidx, uidx)
    push!(plots, p)

    # roll angle, pitch angle, slip angle, understeer angle
    sidx = ["ϕ", "θ", "β", "ψ"]
    label = ["Understeer angle α_u" "Steer angle δ"]
    ylabel = "Angles [°]"
    p = ltiplot(system, yoft, [yoft[system.sidx["α_u-δ"], :] .+ δ δ]; ylabel, label, sidx, uidx)
    push!(plots, p)

    # G lift
    sidx = ["z_G"]
    label = ["Steer angle δ"]
    ylabel = ", δ [°]"
    p = ltiplot(system, yoft, δ; ylabel, label, sidx, uidx)
    push!(plots, p)

    # lateral forces
    yidx = [0]
    aidx = ["Y_lf", "Y_lr", "Y_rf", "Y_rr"]
    ylabel = "Lateral forces [N]"
    p = ltiplot(system, yoft; ylabel, yidx, aidx)
    push!(plots, p)

    # yaw moment
    Nidx = get.([system.sidx], ["N_lf", "N_lr", "N_rf", "N_rr"], 0)
    N = sum(yoft[Nidx, :]; dims=1)[1, :]
    yidx = Nidx
    uidx = [0]
    label = ["Total"]
    ylabel = "Yaw moments [Nm]"
    p = ltiplot(system, yoft, N; ylabel, label, yidx, uidx, formatter=:plain)
    push!(plots, p)

    # plots not directly from inputs or outputs
    uidx = [0]
    yidx = [0]

    # get tire vertical forces
    ZZ = Z0' .- yoft[Zidx, :]'

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
    α = 180 / π * yoft[αidx, :]' - δ .* [1, 0, 1, 0]'

    label = ["Tire slip angle α_lf" "Tire slip angle α_lr" "Tire slip angle α_rf" "Tire slip angle α_rr"]
    ylabel = "Slip angles [°]"
    p = ltiplot(system, yoft, α; ylabel, label, yidx, uidx)
    push!(plots, p)

    # get tire lateral forces
    YY = hcat(yoft.u...)

    acc = sum(YY, dims=1)[1, :] * 9.81 / sum(Z0)
    label = ["ru" "Σf/m" "vdot"]
    ylabel = "Lateral accel'n [m/s^2]"
    p = ltiplot(system, yoft, [yoft[system.sidx["ru"], :] acc acc - yoft[system.sidx["ru"], :]]; ylabel, label, yidx, uidx)
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

    bode = :skip
    impulse = :skip
    ss = :skip
    summarize(system, result; plots, bode, ss, impulse, format)

    # generate animations of the mode shapes
    # animate_modes(system, result, true)

    println("Done.")

end

@time main()
