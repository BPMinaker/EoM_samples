using EoM #, EoM_X3D
using Plots
plotlyjs()

format = :screen
# format = :html

include(joinpath("models", "input_ex_half_car.jl"))

function main()

    m = 2000
    a = 1.5
    b = 1.3
    cf = 100
    cr = 100
    Iy = 2000

    system = input_ex_half_car(; m, a, b, cf, cr, Iy)
    output = run_eom!(system)
    impulse = :skip
    result = analyze(output; impulse)

    summarize(result; format)

    # animate_modes(system, result)

end

println("Starting...")
main()
println("Done.")
