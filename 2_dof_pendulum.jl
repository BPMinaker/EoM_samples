using EoM
# using EoM_X3D

include(joinpath("models", "input_ex_2_dof_pendulum.jl"))

function main()

    system = input_ex_pendulum_2()
    output = run_eom!(system)
    result = analyze(output)

    summarize(system, result)

    # animate_modes(system, result)

end

println("Starting...")
main()
println("Done.")
