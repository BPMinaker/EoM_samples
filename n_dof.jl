using EoM

include(joinpath("models", "input_n_dof.jl"))

system = input_n_dof(n=10)
output = run_eom!(system, true)
result = analyze(output, true)

summarize(system, result)
# summarize(system, result, format = :html)

# using EoM_X3D
# animate_modes(system, result, scale = 0.5)
