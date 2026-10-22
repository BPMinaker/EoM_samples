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
    kr = 25000
    kf = 30000
    cr = 1250
    cf = 1500

#    Iy = 2000
#    kr = a * kf / b
#    Iy = m*a*b

    vpts = 0.5:0.01:1.2
    vpt_name = ["Iy/mab" "Normalized moment of inertia" ""]

    system = [input_ex_bounce_pitch(;m, a, b, kf, kr, cf, cr, Iy=x*m*a*b) for x in vpts]
    # note that we don't pass u here, it is only used later for delay from front to rear axle

    output = run_eom!.(system)
    impulse = :skip
    result = analyze.(output; impulse)

    bode = [1 1; 0 0; 1 1; 1 1; 1 0; 0 1; 1 0; 0 1]
    summarize(vpts, result; vpt_name, bode, impulse, format)

    [i.sys_data.name *= " with input delay" for i in result]
    input_delay!.(result, [(a + b) / u], [[1, 2]]) # this function modifies the frequency response to include the input delay (multiplies second input by exp(-iϕ))
    bode =[1, 0, 1, 1, 0, 0, 1, 1]
    summarize(vpts, result; vpt_name, bode, impulse, format)

#    animate_modes(system, result)

end

println("Starting...")
main()
println("Done.")
