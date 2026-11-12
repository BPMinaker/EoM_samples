using EoM, EoM_X3D
using Plots
plotlyjs()

format = :screen
#format = :html

include(joinpath("models", "input_ex_bounce_pitch.jl"))

function main()

    u = 20

    m = 2000
    a = 1.5
    b = 1.3
    kf = 25000
    kr = 30000
    cf = 1500
    cr = 1600
    Iy = 2000

#    kr = a * kf / b
#    Iy = m*a*b

    system = input_ex_bounce_pitch(;m, a, b, kf, kr, cf, cr, Iy) # note that we don't pass u here, it is only used later for delay
    output = run_eom!(system)
    impulse = :skip
    result = analyze(output; impulse)

    # specify which input-output combinations to plot in Bode plots
    nin = length(result.sys_data.anames)
    nout = length(result.sys_data.snames)
    bode = zeros(nout,nin)

    bode[result.sys_data.sidx["z_G"], result.sys_data.aidx["u_f"]] = 1 
    bode[result.sys_data.sidx["z_G"], result.sys_data.aidx["u_r"]] = 1

    bode[result.sys_data.sidx["θ(a+b)"], result.sys_data.aidx["u_f"]] = 1 
    bode[result.sys_data.sidx["θ(a+b)"], result.sys_data.aidx["u_r"]] = 1

    bode[result.sys_data.sidx["z_P"], result.sys_data.aidx["u_f"]] = 1 
    bode[result.sys_data.sidx["z_P"], result.sys_data.aidx["u_r"]] = 1

    bode[result.sys_data.sidx["z_f-u_f"], result.sys_data.aidx["u_f"]] = 1 
    bode[result.sys_data.sidx["z_r-u_r"], result.sys_data.aidx["u_r"]] = 1

    bode[result.sys_data.sidx["z_f"], result.sys_data.aidx["u_f"]] = 1 
    bode[result.sys_data.sidx["z_r"], result.sys_data.aidx["u_r"]] = 1

    summarize(result; bode, format)

    animate_modes(system, result)

    zofx = random_road(class=5)

    # but we need to convert to time index, where x=ut; assuming a forward speed of u=10 m/s gives
    u_vec(_, t) = [zofx(10 * t), zofx(10 * t - (a + b))]

    println("Solving time history...")
    t1 = 0
    t2 = 10
    yoft = ltisim(result, u_vec, (t1, t2))

    # plot bounce, pitch, passenger motion, and suspension travel vs time
    println("Plotting results...")
    plots = [ltiplot(yoft; sidx = i) for i in [["z_G"], ["θ(a+b)"], ["z_P"], ["z_f-u_f", "z_r-u_r"]]]

    summarize(result; plots, format, tex=true)

    result.sys_data.name *= " with input delay"
    input_delay!(result, (a + b) / u, [1, 2]) # this function modifies the frequency response to include the input delay (multiplies second input by exp(-iϕ))

    # with the front and rear inputs coupled by a time delay of (a+b)/u, we now have only one input, but still the same outputs, so plot only the first column 

    # specify which input-output combinations to plot in Bode plots
    # note that u_r is gone
    nin = length(result.sys_data.anames)
    nout = length(result.sys_data.snames)
    bode = zeros(nout,nin)

    bode[result.sys_data.sidx["z_f"], result.sys_data.aidx["u_f"]] = 1 
    bode[result.sys_data.sidx["z_r"], result.sys_data.aidx["u_f"]] = 1

    summarize(result; bode, ss=:skip, impulse=:skip, format)

end

println("Starting...")
main()
println("Done.")
