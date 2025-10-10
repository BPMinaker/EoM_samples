using EoM
include(joinpath("models", "input_ex_disk.jl"))

function main()

    format = :screen
    # format = :html

    vpts = 0:3/125:3

    system = [input_ex_disk(u=x) for x in vpts]
    output = run_eom!.(system)
    result = analyze.(output; ss=:skip, impulse=:skip, freq=(-1, 1))

    summarize(system, vpts, result; format)

end

println("Starting...")
main()
println("Done.")
