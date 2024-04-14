using EoM
EoM.plotly()

include(joinpath("models", "input_ex_2_dof_pendulum.jl"))

system = input_ex_pendulum_2()
output = run_eom!(system)
result = analyze(output)

summarize(system, result)

# using EoM_X3D
# animate_modes(system, result)

println("Done.")
