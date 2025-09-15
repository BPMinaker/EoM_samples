using EoM
# using EoM_X3D
include(joinpath("models", "input_ex_bicycle_rider.jl"))

function main()

    format = :screen
    # format = :html

    vpts = 0:0.08:10

    system = [input_ex_bicycle_rider(u=x) for x in vpts]
    output = run_eom!.(system, vpts .== 0)
    result = analyze.(output, vpts .== 0; ss=:skip, impulse=:skip)

    summarize(system, vpts, result; format)
    # summarize(system[10], result[10]; format)

    # animate_modes(system[10], result[10])

end

println("Starting...")
main()
println("Done.")
