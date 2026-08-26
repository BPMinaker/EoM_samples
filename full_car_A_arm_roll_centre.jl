using EoM, EoM_X3D
using Plots
plotlyjs()

format = :screen
#format = :html

include(joinpath("models", "input_ex_full_car_A_arm.jl"))

r = 0.315
u = 1.0 # no effect, except can't be zero or linear tire gives division by zero

function main()

    # set the geometry and inertial parameters
    m = 1565
    a = 2.63 * (1 - fwf)
    b = 2.63 * fwf
    tf = 1.8
    tr = 1.8
    hG = 0.57
    Ix = 818 # moments of inertia
    Iy = 3267
    Iz = 3508
    muf = 50 # unsprung mass, front
    mur = 50
    kt = 180000 # tire vertical stiffness
    Iw = 1.75
    cfy = 1437 * 180/π  # front axle cornering stiffness in N/rad
    cry = 1507 * 180/π # rear axle cornering stiffness in N/rad
    params = list(; u, m, a, b, tf, tr, hG, Ix, Iy, Iz, kf, kr, cf, cr, krf, krr, muf, mur, cfy, cry, kt, Iw)

    # build system description with no cornering stiffnesses because will use a nonlinear tire model
    system = input_full_car_a_arm(; params, front, rear) # make sure to include all parameters you want to change here

    item = rigid_point("stationary")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location = [0, 0, hG]
    item.forces = 1
    item.moments = 0
    item.axis = [1, 0, 0]
    add_item!(item, system)

    rc_lock!(system, "LF", a, tf/2)
    rc_lock!(system, "RF", a, -tf/2)
    rc_lock!(system, "LR", -b, tr/2)
    rc_lock!(system, "RR", -b, -tr/2)

    output = run_eom!(system, false)
    result = analyze(output, false; impulse=:skip, bode=:skip)
    summarize(result; format)

    animate_modes(system, result)

    hf = hG + real(result.centre[3]) + real(result.centre[6]) / real(result.centre[4]) * a
    hr = hG + real(result.centre[3]) + real(result.centre[6]) / real(result.centre[4]) * -b

    println("Front roll centre height: ", round(hf; digits=3))
    println("Rear roll centre height: ", round(hr; digits=3))

end

function rc_lock!(system, corner, x, y)
    item = rigid_point(corner * " wheel lock")
    item.body[1] = corner * " wheel"
    item.body[2] = "ground"
    item.location = [x, y, 0]
    item.forces = 2
    item.moments = 0
    item.axis = [1, 0, 0]
    add_item!(item, system)
end

println("Starting...")
# get all the supension and properties and weight distribution
include(joinpath("specifications", "full_car_specs.jl"))
main()
println("Done.")
