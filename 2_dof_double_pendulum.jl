using EoM, EoM_X3D
using Plots
plotlyjs()

# format = :screen
format = :html

include(joinpath("models", "input_ex_2_dof_double_pendulum.jl"))

function main()

    system = input_ex_double_pendulum(; x=0.25)
    output = run_eom!(system)
    result = analyze(output)

    summarize(result; format)

    animate_modes(system, result)

end

println("Starting...")
main()
println("Done.")
