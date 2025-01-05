module two_dof

using EoM

include(joinpath("models", "input_ex_2_dof_double_pendulum.jl"))

system = input_double_pendulum()
output = run_eom!(system)
result = analyze(output)

summarize(system, result, format=:screen)

# using EoM_X3D
# animate_modes(system, result)

end

println("Done.")
