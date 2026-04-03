using EoM, EoM_X3D
using Plots
plotlyjs()

# format = :screen
format = :html

include(joinpath("models", "input_ex_2_dof_pendulum.jl"))

function main()

    system = input_ex_pendulum_2()
    output = run_eom!(system)
    result = analyze(output)

    summarize(result; format)

    animate_modes(system, result)

end

println("Starting...")
main()
println("Done.")
