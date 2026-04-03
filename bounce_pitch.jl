using EoM, EoM_X3D
using Plots
plotlyjs()

#format = :screen
format = :html

include(joinpath("models", "input_ex_bounce_pitch.jl"))

function main()

    u = 20

    m = 2000
    a = 1.5
    b = 1.3
    kf = 25000
    kr = 30000
    cf = 0
    cr = 0
    Iy = 2000

#    kr = a * kf / b
#    Iy = m*a*b

    system = input_ex_bounce_pitch(; m, a, b, kf, kr, cf, cr, Iy) # note that we don't pass u here, it is only used later for delay
    output = run_eom!(system)
    impulse = :skip
    result = analyze(output; impulse)

    # ask for Bode plots of with both front and rear inputs to both bounce and pitch and passenger motion outputs, but ignore the suspension travel outputs 
    bode = [1 1; 1 1; 1 1; 0 0; 0 0]
    summarize(result; bode, format)

    animate_modes(system, result)

    zofx = random_road(class=5)

    # but we need to convert to time index, where x=ut; assuming a forward speed of u=10 m/s gives
    u_vec(_, t) = [zofx(10 * t), zofx(10 * t - (a + b))]

    println("Solving time history...")
    t1 = 0
    t2 = 10
    yoft = ltisim(result, u_vec, (t1, t2))

    # plot bounce
    sidx = ["z_G"]
    p1 = ltiplot(yoft; sidx)

    # plot pitch
    sidx = ["θ(a+b)"]
    p2 = ltiplot(yoft; sidx)

    # plot passenger motion
    sidx = ["z_P"]
    p3 = ltiplot(yoft; sidx)

    # plot suspension travel
    sidx = ["z_f", "z_r"]
    p4 = ltiplot(yoft; sidx)

    plots = [p1, p2, p3, p4]
    summarize(result; plots, format)

    # regenerate eqns of motion for coupled input analysis
    system = input_ex_bounce_pitch(; m, a, b, kf, kr, cf, cr, Iy) # note that we don't pass u here, it is only used later for delay
    system.name *= " with input delay"
    output = run_eom!(system)
    result = analyze(output; impulse)
    input_delay!(result, (a + b) / u, [1, 2]) # this function modifies the equations of motion to include the input delay (multiplies second input by exp(-iϕ))

    # with the front and rear inputs coupled by a time delay of (a+b)/u, we now have only one input, but still the same outputs, so keep only the first column of the previous
    bode = bode[:, 1]
    ss = :skip
    summarize(result; bode, ss, impulse, format)

end

println("Starting...")
main()
println("Done.")
