using EoM, EoM_X3D

include(joinpath("models", "input_ex_hanging_chain.jl"))

system = input_ex_hanging_chain()
output = run_eom!(system)
result = analyze(output)

ω = minimum(result.omega_n)
u_vec(_, t) = 0.2 * [sin(2π * ω * t), sin(2π * ω * (t+0.5)), sin(2π * ω * (t+1))]

t1 = 0
t2 = 20
yoft = ltisim(result, u_vec, (t1, t2))

plots = [ltiplot(system, yoft)]
summarize(system, result; plots)



# summarize(system, result; plots, format = :html)
# animate_modes(system, result)

system = input_ex_hanging_chain()
sensors_animate!(system)
output = run_eom!(system)
result = analyze(output)

y = ltisim(result, u_vec, (t1, t2))
animate_history(system, y.t, y)


####

include(joinpath("models", "input_ex_planar_loops.jl"))

system = input_ex_planar_loops()
output = run_eom!(system)
result = analyze(output)

summarize(system, result)

# summarize(system, result; format = :html)
animate_modes(system, result)

#####

include(joinpath("models", "input_ex_pyramid.jl"))

system = input_ex_pyramid()
output = run_eom!(system)
result = analyze(output)

summarize(system, result)

# summarize(system, result; format = :html)
animate_modes(system, result)


println("Done.")

