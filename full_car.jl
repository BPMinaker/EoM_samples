using EoM #, EoM_X3D
using Plots
plotlyjs()

format = :screen
# format = :html

include(joinpath("models", "input_ex_full_car.jl"))

function main()

    m = 2000
    a = 2.65 * 0.58
    b = 2.65 * 0.42
    tf = 1.71
    tr = 1.71
    cf = 100
    cr = 100
    Iy = 2000

    system = input_ex_full_car(; m, a, b, tf, tr, cf, cr, Iy)
    output = run_eom!(system)
    result = analyze(output)

    impulse = :skip
    summarize(result; impulse, format)

    # animate_modes(system, result)

end

println("Starting...")
main()
println("Done.")
