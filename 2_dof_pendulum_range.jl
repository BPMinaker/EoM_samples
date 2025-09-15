using EoM
# using EoM_X3D

include(joinpath("models", "input_ex_2_dof_pendulum.jl"))

function main()

    vpts = 0.1:0.01:1
    vpt_name = ["l" "Length" "m"]

    format = :screen
    # format = :html

    system = [input_ex_pendulum_2(l=x) for x in vpts]
    output = run_eom!.(system)
    result = analyze.(output; ss=:skip)

    summarize(system, vpts, result; vpt_name, format)

end

println("Starting...")
main()
println("Done.")
