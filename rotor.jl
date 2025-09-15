using EoM
include(joinpath("models", "input_ex_rotor.jl"))

function main()

    vpts = 0.0:2pi/125:2pi
    vpt_name = ["r" "Angular speed" "rad/s"]

    system = [input_ex_rotor(; r=x) for x in vpts]
    output = run_eom!.(system, vpts .== 0)
    result = analyze.(output, vpts .== 0; ss=:skip, impulse=:skip)

    bode = [1 0; 1 0; 0 1]
    summarize(system, vpts, result; bode, vpt_name)

end

println("Starting...")
main()
println("Done.")
