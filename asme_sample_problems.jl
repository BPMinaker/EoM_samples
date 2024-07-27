using EoM, EoM_X3D

include(joinpath("models", "input_ex_hanging_chain.jl"))

system = input_ex_hanging_chain()
output = run_eom!(system)
result = analyze(output)

t = 0:0.005:20
ω = minimum(result.omega_n)
u_vec(~, t) = 0.2 * [sin(2π * ω * t), sin(2π * ω * (t+0.5)), sin(2π * ω * (t+1))]
y = splsim(result.ss_eqns, u_vec, t)
f = u_vec.(0, t)

xlabel = "Time [s]"
ylabel = "x [m], y [m], ψ [rad]"
label = ["x" "y" "ψ"]
lw = 2
size = (800, 400)

plots = [plot(t, [Matrix(y) Matrix(StateSpaceSet(f))]; xlabel, ylabel, label, lw, size)]
summarize(system, result; plots)

system = input_ex_hanging_chain()
sensors_animate!(system)
output = run_eom!(system)
result = analyze(output)

y = splsim(result.ss_eqns, u_vec, t)

p = Matrix(y)'
animate_history(system, p, t)


# summarize(system, result; plots, format = :html)
# animate_modes(system, result)


#=
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

=#

println("Done.")

