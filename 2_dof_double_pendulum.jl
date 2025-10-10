using EoM
using EoM_X3D
include(joinpath("models", "input_ex_2_dof_double_pendulum.jl"))

# format = :screen
format = :html

function main()

    system = input_ex_double_pendulum(; x=0.25)
    output = run_eom!(system)
    result = analyze(output)

    summarize(system, result; format)

    animate_modes(system, result)

end

println("Starting...")
main()
println("Done.")
