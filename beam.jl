using EoM
# using EoM_X3D

include(joinpath("models", "input_ex_beam.jl"))

function main()

    format = :screen
    # format = :html

    mpi = 0.0254
    h = 2 * mpi
    b = 1 * mpi
    t = 0.060 * mpi
    E = 200e9 # Pa
    ρ = 7850  # kg/m^3

    I1 = (b * h^3 - (b - 2 * t) * (h - 2 * t)^3) / 12
    I2 = (b^3 * h - (b - 2 * t)^3 * (h - 2 * t)) / 12

    EI1 = E * I1
    EI2 = E * I2
    mpul = ρ * (b * h - (b - 2 * t) * (h - 2 * t))

    l = 1
    n = 1

    system = input_ex_beam(; EI1, EI2, mpul, l, n)
    output = run_eom!(system, true)
    result = analyze(output, true)

    summarize(system, result; format)

    # animate_modes(system, result)

end

println("Starting...")
main()
println("Done.")
