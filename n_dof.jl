using EoM, EoM_X3D
using Plots
plotlyjs()

# format = :screen
format = :html

include(joinpath("models", "input_n_dof.jl"))

function main()

    system = input_n_dof(n=10)
    output = run_eom!(system, true)
    result = analyze(output, true)

    summarize(result; format)
    animate_modes(system, result, scale = 0.5)

end

println("Starting...")
main()
println("Done.")
