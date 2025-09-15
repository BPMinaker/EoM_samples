using EoM
include(joinpath("models", "input_ex_1_dof_pendulum.jl"))

function main()

    system = input_ex_pendulum()
    output = run_eom!(system)
    result = analyze(output)

    summarize(system, result)

end

println("Starting...")
main()
println("Done.")
