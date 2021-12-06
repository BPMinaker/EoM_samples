using EoM

build_examples()
include(joinpath("examples", "input_n_dof.jl"))

system = input_n_dof(n = 6)
output = run_eom!(system, true)
result = analyze(output, true)

summarize(system, result)

#using EoM_X3D
#animate_modes(system, result(), scale = 0.5)
