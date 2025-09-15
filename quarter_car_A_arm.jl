using EoM, EoM_X3D

include(joinpath("models", "input_quarter_car_a_arm_pushrod.jl"))
include(joinpath("models", "susp.jl"))
include(joinpath("models", "tire.jl"))

function main()

    # here you can enter your vehicle specs by name
    a = 2.65 * 0.58
    tw = 1.71
    r = 0.346
    u = 10

    format = :screen
    # format = :html

    system = quarter_car_a_arm_pushrod(; u, a, tw, r)
    sensors_animate!(system)
    output = run_eom!(system)
    result = analyze(output; impulse = :skip)

    zofx = random_road(class=5, L=200)
    u_vec(_, t) = [zofx(u * t)]

    t1 = 0
    t2 = 20
    yoft = ltisim(result, u_vec, (t1, t2))

    println("Plotting results...")
    # plot sprung mass
    sidx = ["z_s"]
    p1 = ltiplot(system, yoft; sidx)
    # at 1000 Hz, a time interval of 10 s gives us 10000 points, which is fine until we want to plot on a screen with only 1920 pixels, so by default we downsample to plot a maximum 2000 points, unless you set an intger variable scale = x in the `ltiplot` call, where x is the number of points to skip; for example, `ltiplot(system, yoft; yidx, scale=50)` will plot every 50th point, or 200 points in total if the time interval is 10 s at 1000 Hz

    # plot suspension travel
    sidx = ["z_s-z_u"]
    p2 = ltiplot(system, yoft; sidx)

    # plot tire compression
    sidx = ["z_u-z_g"]
    p3 = ltiplot(system, yoft; sidx)

    uidx = [0]
    sidx = ["f_s", "f_d", "f_i"]
    p4 = ltiplot(system, yoft; uidx, sidx)

    plots = [p1, p2, p3, p4]

    summarize(system, result; plots, format)

    #animate_modes(system, result)
    #eom_draw(system)

    animate_history(system, yoft)

end

println("Starting...")
main()
println("Done.")
