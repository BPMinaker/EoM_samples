using EoM

include(joinpath("models", "input_ex_1_dof_pendulum.jl"))

system = input_ex_pendulum()
output = run_eom!(system)
result = analyze(output)

impulse = :skip
summarize(system, result; impulse)

println("Done.")
