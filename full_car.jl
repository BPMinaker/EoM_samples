using EoM
# using EoM_X3D
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

    format = :screen
    # format = :html

    system = input_ex_full_car(; m, a, b, tf, tr, cf, cr, Iy)
    output = run_eom!(system)
    result = analyze(output)

    impulse = :skip
    summarize(system, result; impulse, format)

    # animate_modes(system, result)
    println("Done.")

end

main()
