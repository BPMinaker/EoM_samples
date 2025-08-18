using EoM
include(joinpath("models", "input_ex_top.jl"))

function main()

    vpts = 0:10/125:10
    vpt_name = ["r" "Angular speed" "rad/s"]

    system = [input_ex_top(r=x) for x in vpts]
    output = run_eom!.(system)
    result = analyze.(output; ss=:skip, impulse=:skip)

    summarize(system, vpts, result; vpt_name)

    println("Done.")

end

main()
