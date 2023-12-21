using EoM, EoM_X3D

include(joinpath("models", "input_ex_hanging_chain.jl"))

system = input_ex_hanging_chain()
output = run_eom!(system)
result = analyze(output)

summarize(system, result)

# summarize(system, result; format = :html)
# animate_modes(system, result)

####

include(joinpath("models", "input_ex_planar_loops.jl"))

system = input_ex_planar_loops()
output = run_eom!(system)
result = analyze(output)

summarize(system, result)

# summarize(system, result; format = :html)
# animate_modes(system, result)

#####

include(joinpath("models", "input_ex_pyramid.jl"))

system = input_ex_pyramid()
output = run_eom!(system)
result = analyze(output)

summarize(system, result)

# summarize(system, result; format = :html)
# animate_modes(system, result)

println("Done.")
