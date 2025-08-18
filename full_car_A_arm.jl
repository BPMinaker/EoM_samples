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

    system = input_full_car_a_arm_pushrod(; u, a, b, tw, r)
    output = run_eom!(system)
    result = analyze(output; impulse = :skip, freq=(-1, 2))

    lfidx = system.aidx["u_LF"]
    lridx = system.aidx["u_LR"]
    rfidx = system.aidx["u_RF"]
    rridx = system.aidx["u_RR"]

    zofxl, zofxr = random_road(class=5, dz=0.2)
    function u_vec(_, t)
        uu = zeros(length(system.aidx)) 
       
        uu[lfidx] = zofxl(u * t)
        uu[rfidx] = zofxr(u * t)
        uu[lridx] = zofxl(u * t - a - b)
        uu[rridx] = zofxr(u * t - a - b)

        uu
    end

    println("Solving time history...")
    t1 = 0
    t2 = 20
    yoft = ltisim(result, u_vec, (t1, t2))

    aidx = ["u_LF"]

    sidx = ["z_LFc"]
    p1 = ltiplot(system, yoft; sidx, aidx)

    sidx = ["z_LFc-z_LF"]
    p2 = ltiplot(system, yoft; sidx, aidx)

    sidx = ["z_LF-u_LF"]
    p3 = ltiplot(system, yoft; sidx, aidx)

    plots = [p1, p2, p3]

    summarize(system, result; plots, format)

    println("Done.")

end

main()
