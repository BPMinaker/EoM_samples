using Base: format_bytes
using EoM
include(joinpath("models", "input_ex_disk.jl"))

function main()

    vpts = 0:3/125:3
    format = :screen
    # format = :html

    system = [input_ex_disk(u=x) for x in vpts]
    output = run_eom!.(system)
    result = analyze.(output; freq=(-1, 1))

    summarize(system, vpts, result; ss=:skip, impulse=:skip, format)

    println("Done.")

end

main()
