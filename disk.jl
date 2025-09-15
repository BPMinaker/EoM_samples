using Base: format_bytes
using EoM
include(joinpath("models", "input_ex_disk.jl"))

function main()

    vpts = 0:3/125:3
    format = :screen
    # format = :html

    system = [input_ex_disk(u=x) for x in vpts]
    output = run_eom!.(system)
    result = analyze.(output; ss=:skip, impulse=:skip, freq=(-1, 1))

    summarize(system, vpts, result; format)

end

println("Starting...")
main()
println("Done.")
