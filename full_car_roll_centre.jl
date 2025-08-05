using EoM
# using EoM_X3D
include(joinpath("models", "input_ex_roll_centre.jl"))
# note that this model has four actuators, one for each tire lateral force, so we can use an external calculation for the tire model, anything we like, e.g. a magic formula

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

    format = :screen
    # format = :html

    # build system description with no cornering stiffnesses because will use a nonlinear tire model
    system = input_full_car_rc(; m, u, a, b, cfy, cry, hf, hr, kf, kr, krf, krr, r) # make sure to include all parameters here, and again below!!!
    output = run_eom!(system, true)
    result = analyze(output, true)

    # define a steer function, use a slowly increasing steer angle, in units of degrees, i.e., 3 degrees after 30 seconds
    steer(t) = t / 10

    # get static tire normal loads (kN)
    tires = (get.([system.flex_points_name], ["LF tire, Z", "LR tire, Z", "RF tire, Z", "RR tire, Z"], 0))
    Z0 = vcat(getfield.(tires, :preload)...)

    # find the output indices for tire normal loads and slip angles
    Zidx = get.([system.sidx], ["Z_lf", "Z_lr", "Z_rf", "Z_rr"], 0)
    αidx = get.([system.sidx], ["α_lf", "α_lr", "α_rf", "α_rr"], 0)

    # compute applied tire force
    function u_vec(x, t)
        # get sensor outputs (D=0)
        y = result.ss_eqns.C * x
        # get total normal load
        Z = Z0 - y[Zidx]
        # get slip angles from sensors, subtract steer on front, correct for units
        α = y[αidx] .- steer(t) * [1, 0, 1, 0] * π / 180
        α .*= [1, 1, -1, -1] # flip sign on RF and RR slip angles (modified iso sign convention)

        # compute tire force, ignore camber effect, restroing moment
        Y = tire(Z, α, [0, 0, 0, 0])[1]
        Y .* [-1, -1, 1, 1] # flip sign on LF and LR tire forces (mirror)
    end

    println("Solving time history...")
    t1 = 0
    t2 = 30

    yoft = ltisim(result, u_vec, (t1, t2))
    δ = steer.(yoft.t)

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
    sidx = ["ϕ", "θ", "β"]
    label = ["Understeer angle α_u" "Steer angle δ"]
    ylabel = "Angles [°]"
    p = ltiplot(system, yoft, [yoft[18, :] .+ δ δ]; ylabel, label, sidx, uidx)
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
    # find total front and rear tire forces
    YF = (YY[1, :] + YY[3, :])
    YR = (YY[2, :] + YY[4, :])

    # get tire forces using slip but with static normal loads, remember to flip signs because of sign convention, mirror
    α[:, 3:4] *= -1
    Y00 = hcat([tire.(Z0, i * π / 180, [0, 0, 0, 0]) for i in eachrow(α)]...)
    Y0 = [i[1] for i in Y00]
    Y0[1:2, :] *= -1

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

    println("Plotted results.")

    #=
    # compute cornering stiffnesses from the magic tire model
    cfy = mtm[4] * sin.(2 * atan.(Z0[1] / mtm[5]))
    cry = mtm[4] * sin.(2 * atan.(Z0[2] / mtm[5]))

    # rebuild the equations of motion using the updated cornering stiffnesses
    # really only need to do this if we want to see the effect of the cornering stiffnesses on the yaw eigenvalues
    system = input_full_car_rc(; m, u, a, b, cfy, cry, hf, hr, kf, kr, krf, krr)
    output = run_eom!(system, false)
    result = analyze(output, false)
    =#

    bode = :skip
    impulse = :skip
    ss = :skip
    summarize(system, result; plots, bode, ss, impulse, format)

    # generate animations of the mode shapes
    # animate_modes(system, result, true)

    println("Done.")

end

main()
