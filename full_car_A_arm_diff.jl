using EoM, EoM_X3D
using Plots
plotlyjs()

format = :screen
#format = :html

include(joinpath("models", "input_ex_full_car_A_arm_diff.jl"))

r = 0.315
u = 22.4 # forward speed in m/s (50 mph)
t1 = 0
t2 = 10

dtr = π / 180

function main()

    # set the parameters that can't be adjusted by the student
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
    cfy = 1437 / dtr # front axle cornering stiffness in N/rad
    cry = 1507 / dtr # rear axle cornering stiffness in N/rad
    params = list(; u, m, a, b, tf, tr, hG, Ix, Iy, Iz, kf, kr, cf, cr, krf, krr, muf, mur, cfy, cry, kt, Iw)

    # build system description with no cornering stiffnesses because will use a nonlinear tire model
    system = input_full_car_a_arm_diff(; params, front, rear) # make sure to include all parameters you want to change here
    output = run_eom!(system, true)
    result = analyze(output, true; impulse=:skip)
    summarize(result; format)

    animate_modes(system, result)

end

println("Starting...")
flist = readdir("specifications"; join=true)

for file in flist
    println("Including file: ", file)
    include(file)
    println("Running $team_name's simulation...")

    main()
end

println("Done.")
