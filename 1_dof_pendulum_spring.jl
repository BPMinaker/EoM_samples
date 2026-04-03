using EoM, EoM_X3D
using Plots
plotlyjs()

include(joinpath("models", "input_ex_1_dof_pendulum_spring.jl"))

function main()

    system = input_ex_pendulum_spring()
    output = run_eom!(system)
    result = analyze(output)

    summarize(result)

    animate_modes(system, result)

end

println("Starting...")
main()
println("Done.")
