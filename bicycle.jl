using EoM #, EoM_X3D
using Plots
plotlyjs()

format = :screen
# format = :html

include(joinpath("models", "input_ex_bicycle_rider.jl"))

function main()


    vpts = 0:0.08:10

    system = [input_ex_bicycle_rider(u=x) for x in vpts]
    output = run_eom!.(system, vpts .== 0)
    result = analyze.(output, vpts .== 0; ss=:skip, impulse=:skip)

    summarize(vpts, result; format)
    # summarize(result[10]; format)

    # animate_modes(system[10], result[10])

end

println("Starting...")
main()
println("Done.")
