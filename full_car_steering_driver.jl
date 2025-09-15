using EoM, EoM_X3D
include(joinpath("models", "input_full_car_steering.jl"))
include(joinpath("models", "track.jl"))
include(joinpath("models", "driver.jl"))

r = 0.315
include("my_specs.jl")

function main()

    u = 80 / 3.6

    l = 2.63
    b = fwf * l
    a = l - b
    tf = 1.52
    tr = 1.52
    m = 1571
    muf = 50
    mur = 50
    hG = 0.439
    Ix = 461
    Iy = 1848
    Iz = 2045
    Iw = 1.75

    format = :screen
    # format = :html

    system = input_full_car_steering(; u, a, b, m, muf, mur, hG, Ix, Iy, Iz, Iw, tf, tr, kf, kr, cf, cr, krf, krr, front, rear)
    sensors_animate!(system)
    output = run_eom!(system)
    #    eom_draw(system)
    bode = :skip
    impulse = :skip
    ss = :skip
    result = analyze(output; ss, bode, impulse)

    # get static tire normal loads (kN)
    Z0 = [
        system.flex_points_name["LF Tire, Z"].preload[1],
        system.flex_points_name["LR Tire, Z"].preload[1],
        system.flex_points_name["RF Tire, Z"].preload[1],
        system.flex_points_name["RR Tire, Z"].preload[1]
    ]

    Zsidx = get.([system.sidx], ["Zs_lf", "Zs_lr", "Zs_rf", "Zs_rr"], 0)
    Zdidx = get.([system.sidx], ["Zd_lf", "Zd_lr", "Zd_rf", "Zd_rr"], 0)
    uuidx = get.([system.sidx], ["u_lf", "u_lr", "u_rf", "u_rr"], 0)
    vidx = get.([system.sidx], ["v_lf", "v_lr", "v_rf", "v_rr"], 0)
    δidx = get.([system.sidx], ["δ_lf", "δ_lr", "δ_rf", "δ_rr"], 0)
    γidx = get.([system.sidx], ["γ_lf", "γ_lr", "γ_rf", "γ_rr"], 0)

    Yidx = get.([system.aidx], ["Y_lf", "Y_lr", "Y_rf", "Y_rr"], 0)
    Nidx = get.([system.aidx], ["N_lf", "N_lr", "N_rf", "N_rr"], 0)
    Lidx = system.aidx["L"]

    # compute applied tire force
    function u_vec(x, t)
        # get sensor outputs (D=0)
        y = result.ss_eqns.C * x
        # get total normal load
        Z = Z0 - y[Zsidx] - y[Zdidx]

        # get vehicle location and heading from sensors (y is the output vector)
        offset = y[system.sidx["y"]]
        heading = y[system.sidx["ψ"]]
        δ = driver(l, offset, heading, u * t)

        # implement steering control law
        fr = 8000 * (0.5 * y[system.sidx["δ_lf"]] + 0.5 * y[system.sidx["δ_rf"]] - δ)

        # get slip angles from sensors, subtract steer
        α = y[vidx] ./ (y[uuidx] .+ u) - y[δidx]
        α .*= [1, 1, -1, -1] # flip sign on RF and RR slip angles (modified iso sign convention)

        γ = y[γidx] # camber angles
        γ .*= [1, 1, -1, -1] # flip sign on RF and RR camber angles

        # compute tire force
        # flip sign on LF and LR forces (mirror of right hand side)
        G = tire(Z, α, γ)

        uu = zeros(length(system.aidx))
        uu[Yidx] = G[1] .* [-1, -1, 1, 1]
        uu[Nidx] = G[2] .* [-1, -1, 1, 1]
        uu[Lidx] = fr

        uu
    end

    println("Solving time history...")
    t1 = 0
    t2 = 15
    yoft = ltisim(result, u_vec, (t1, t2))

    animate_history(system, yoft)

    # go back and recompute what the steer angle was, which is the output of the driver model (it wasn't recorded during the simulation because it is not an input or output of the system, the input is the tire force)
    δ = driver.(l, yoft[system.sidx["y"], :], yoft[system.sidx["ψ"], :], u * yoft.t)

    println("Plotting results...")
    # empty plot vector to push plots into
    plots = []

    uidx = [0]
    sidx = ["r"]
    label = ["Target steer angle δ"]
    ylabel = ", δ [rad]"
    p = ltiplot(system, yoft, δ; ylabel, label, sidx, uidx)
    push!(plots, p)

    # plot the steer moment
    yidx = [0]
    aidx = ["L"]
    p = ltiplot(system, yoft; yidx, aidx)
    push!(plots, p)

    yidx = δidx
    label = ["Target steer angle δ"]
    ylabel = "Steer angles [rad]"
    p = ltiplot(system, yoft, δ; ylabel, label, yidx, uidx)
    push!(plots, p)

    sidx = ["ϕ", "θ", "ψ", "β"]
    ylabel = "Angles [rad]"
    label = ["Understeer angle α_u"]
    p = ltiplot(system, yoft, yoft[system.sidx["α_u-δ"], :] + 0.5 * yoft[system.sidx["δ_lf"], :] + 0.5 * yoft[system.sidx["δ_rf"], :]; ylabel, label, sidx, uidx)
    push!(plots, p)

    yidx = γidx
    ylabel = "Camber [rad]"
    p = ltiplot(system, yoft; ylabel, yidx, uidx)
    push!(plots, p)

    yidx = [0]
    uidx = [0]

    α = (yoft[vidx, :] ./ (yoft[uuidx, :] .+ u) - yoft[δidx, :])
    αt = copy(α')

    ylabel = "Slip angles [rad]"
    label = ["Slip angle α_lf" "Slip angle α_lr" "Slip angle α_rf" "Slip angle α_rf"]
    p = ltiplot(system, yoft, αt; ylabel, label, yidx, uidx)
    push!(plots, p)

    # get tire vertical forces
    ZZ = Z0' .- yoft[Zsidx, :]' - yoft[Zdidx, :]'
    label = ["Tire force Z_lf" "Tire force Z_lr" "Tire force Z_rf" "Tire force Z_rr"]
    ylabel = "Vertical forces [N]"
    p = ltiplot(system, yoft, ZZ; ylabel, label, yidx, uidx)
    push!(plots, p)

    # lateral forces
    aidx = ["Y_lf", "Y_lr", "Y_rf", "Y_rr"]
    ylabel = "Lateral forces [N]"
    p = ltiplot(system, yoft; ylabel, aidx, yidx)
    push!(plots, p)

    aidx = ["N_lf", "N_lr", "N_rf", "N_rr"]
    ylabel = "Aligning moments [Nm]"
    p = ltiplot(system, yoft; ylabel, aidx, yidx)
    push!(plots, p)

    sidx = ["z"]
    p = ltiplot(system, yoft; sidx, uidx)
    push!(plots, p)

    # find weight transfer
    ΔZ = 0.5 * [ZZ[:, 3] - ZZ[:, 1] ZZ[:, 4] - ZZ[:, 2]]

    label = ["Front weight transfer" "Rear weight transfer"]
    ylabel = "Lateral weight transfer [N]"
    p = ltiplot(system, yoft, ΔZ; ylabel, label, yidx, uidx)
    push!(plots, p)

    # get tire lateral forces
    YY = hcat(yoft.u...)[1:4, :] # get first 4 inputs, which are the tire lateral forces

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
    path = [i[1] for i in track.(x)]
    p = EoM.plot(x, [yoft[system.sidx["y"], :] path]; xlabel, ylabel, label, lw, size)
    push!(plots, p)

    println("Plotted results.")

    summarize(system, result; plots, format)

    # generate animations of the mode shapes
    # animate_modes(system, result, true)

end

println("Starting...")
@time main()
println("Done.")
