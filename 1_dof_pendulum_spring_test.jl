using EoM, EoM_X3D
include(joinpath("models", "input_ex_1_dof_pendulum_spring.jl"))

function main()

    system = input_ex_pendulum_spring()
    output, test = diagnose!(system)
    display(propertynames(test))
    display(test.mass)
    display(test.stiffness)
    display(test.tangent_stiffness)
    display(test.load_stiffness)
    display(test.constraint)
    display(test.deflection)

    result = analyze(output)

    summarize(system, result)

end

println("Starting...")
main()
println("Done.")
