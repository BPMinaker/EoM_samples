using EoM, EoM_X3D
include(joinpath("models", "input_ex_GSXR.jl"))

function main()

    format = :html

    u = 20.0 # speed in m/s
    system = input_ex_GSXR(; u)
    output = run_eom!(system)
    result = analyze(output)

    ss = :skip
    summarize(system, result; ss, format)
    #animate_modes(system, result)

end

println("Starting...")
main()
println("Done.")
