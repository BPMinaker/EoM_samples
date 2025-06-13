using EoM
# using EoM_X3D
include(joinpath("models", "input_ex_bricard.jl"))

function main()

    system = input_ex_bricard(m=2, l=0.4)
    output = run_eom!(system)
    result = analyze(output)

    summarize(system, result)

    # animate_modes(system, result)

    println("Done.")

end

main()
