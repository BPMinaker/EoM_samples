using EoM
# using EoM_X3D
include(joinpath("models", "input_ex_2_dof_double_pendulum.jl"))

function main()

    vpts = 0:0.005:0.45
    vpt_name = ["x" "Fraction" "[]"]

    format = :screen
    # format = :html

    system = [input_double_pendulum(;x) for x in vpts]
    output = run_eom!.(system)
    result = analyze.(output; ss=:skip)

    summarize(system, vpts, result; vpt_name, format)


    # animate_modes(system, result)

    println("Done.")

end

main()
