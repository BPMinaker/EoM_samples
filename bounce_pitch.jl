using EoM
include(joinpath("models", "input_ex_bounce_pitch.jl"))

function main()

    u = 20

    m = 2000
    a = 1.5
    b = 1.3
    kf = 25000
    kr = 30000
    cf = 600
    cr = 500
    Iy = 2000

    #kr = a * kf / b
    #Iy = m*a*b

    format = :screen
    # format = :html

    system = input_ex_bounce_pitch(; m, a, b, kf, kr, cf, cr, Iy) # note that we don't pass u here, it is only used later for delay
    output = run_eom!(system)
    impulse = :skip
    result = analyze(output; impulse)

    # ask for Bode plots of with both front and rear inputs to both bounce and pitch and passenger motion outputs, but ignore the suspension travel outputs 
    bode = [1 1; 1 1; 1 1; 0 0; 0 0]
    summarize(system, result; bode, format)

    # with the front and rear inputs coupled by a time delay of (a+b)/u, we now have only one input, but still the same outputs, so keep only the first column of the previous
    bode = bode[:, 1]
    ss = :skip
    # this function modifies the equations of motion to include the input delay (multiplies second input by exp(-iϕ))
    input_delay!(system, result, (a + b) / u, [1, 2])
    system.name *= " with input delay"
    summarize(system, result; bode, ss, impulse, format)

    # using EoM_X3D
    # animate_modes(system, result)

    system = input_ex_bounce_pitch(; m, a, b, kf, kr, cf, cr, Iy)
    output = run_eom!(system)
    bode  = :skip
    result = analyze(output; ss, bode, impulse)
    system.name *= " time history"

    zofx = random_road(class=5)

    # but we need to convert to time index, where x=ut; assuming a forward speed of u=10 m/s gives
    u_vec(_, t) = [zofx(10 * t), zofx(10 * t - (a + b))]

    println("Solving time history...")
    t1 = 0
    t2 = 10
    yoft = ltisim(result, u_vec, (t1, t2))

    # plot bounce
    sidx = ["z_G"]
    p1 = ltiplot(system, yoft; sidx)

    # plot pitch
    sidx = ["θ(a+b)"]
    p2 = ltiplot(system, yoft; sidx)

    # plot passenger motion
    sidx = ["z_P"]
    p3 = ltiplot(system, yoft; sidx)

    # plot suspension travel
    sidx = ["z_f", "z_r"]
    p4 = ltiplot(system, yoft; sidx)

    plots = [p1, p2, p3, p4]
    summarize(system, result; plots, format)

end

println("Starting...")
main()
println("Done.")
