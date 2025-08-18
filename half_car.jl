using EoM
# using EoM_X3D
include(joinpath("models", "input_ex_half_car.jl"))

function main()

    m = 2000
    a = 1.5
    b = 1.3
    cf = 100
    cr = 100
    Iy = 2000

    format = :screen
    # format = :html

    system = input_ex_half_car(; m, a, b, cf, cr, Iy)
    output = run_eom!(system)
    impulse = :skip
    result = analyze(output; impulse)

    summarize(system, result; format)

    # animate_modes(system, result)

    println("Done.")

end

main()
