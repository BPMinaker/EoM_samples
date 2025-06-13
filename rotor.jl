using EoM
include(joinpath("models", "input_ex_rotor.jl"))

function main()

    vpts = 0.0:2pi/125:2pi
    vpt_name = ["r" "Angular speed" "rad/s"]

    system = [input_ex_rotor(; r=x) for x in vpts]
    output = run_eom!.(system, vpts .== 0)
    result = analyze.(output, vpts .== 0; freq=(-2, 1))

    bode = [1 0; 1 0; 0 1]
    ss = :skip
    impulse = :skip
    summarize(system, vpts, result; bode, ss, impulse, vpt_name)

    println("Done.")

end

main()
