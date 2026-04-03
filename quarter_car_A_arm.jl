using EoM, EoM_X3D
using Plots
plotlyjs()

format = :screen
# format = :html

include(joinpath("models", "input_quarter_car_a_arm_pushrod.jl"))
include(joinpath("models", "susp.jl"))
include(joinpath("models", "tire.jl"))

function main()

    # here you can enter your vehicle specs by name
    a = 2.65 * 0.58
    tw = 1.71
    r = 0.346
    u = 10

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
    p1 = ltiplot(yoft; sidx)

    # plot suspension travel
    sidx = ["z_s-z_u"]
    p2 = ltiplot(yoft; sidx)

    # plot tire compression
    sidx = ["z_u-z_g"]
    p3 = ltiplot(yoft; sidx)

    plots = [p1, p2, p3]

    summarize(result; plots, format)

    #animate_modes(system, result)
    #eom_draw(system)

    animate_history(system, yoft)

end

println("Starting...")
main()
println("Done.")
