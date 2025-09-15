using EoM
# using EoM_X3D
include(joinpath("models", "input_ex_2_dof_double_pendulum.jl"))

function main()

    vpts = 0.16:0.003:0.49
    vpt_name = ["x" "Fraction" "[]"]

    format = :screen
    # format = :html

    system = [input_ex_double_pendulum(;x) for x in vpts]
    output = run_eom!.(system)
    result = analyze.(output; ss=:skip)

    summarize(system, vpts, result; vpt_name, format)

end

println("Starting...")
main()
println("Done.")
