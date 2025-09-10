using EoM, EoM_X3D
include(joinpath("models", "input_ex_2_dof_five_bar_pendulum.jl"))

function main()

    system = input_ex_five_bar_pendulum()
    output = run_eom!(system)
    result = analyze(output)

    summarize(system, result)

    animate_modes(system, result)

    println("Done.")

end

main()
