using EoM, EoM_X3D

include(joinpath("models", "input_full_car_a_arm_pushrod.jl"))
include(joinpath("models", "susp.jl"))
include(joinpath("models", "tire.jl"))
include(joinpath("models", "drive.jl"))

function main()

    # here you can enter your vehicle specs by name, and set the speed
    a = 2.65 * 0.58
    b = 2.65 * 0.42
    tw = 1.94 - 0.23
    r = 0.346
    u = 10

    format = :screen
    # format = :html

    # build the system for the time history animation
    system = input_full_car_a_arm_pushrod(; u, a, b, tw, r)
    sensors_animate!(system)
    output = run_eom!(system)
    result = analyze(output)

    println("Solving time history...")
    t1 = 0
    t2 = 20
    y = ltisim(result, u_vec, (t1, t2))

    animate_history(system, y.t, y)

    println("Done.")

end

main()
