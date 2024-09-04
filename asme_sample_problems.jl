using EoM, EoM_X3D

include(joinpath("models", "input_ex_hanging_chain.jl"))

system = input_ex_hanging_chain()
output = run_eom!(system)
result = analyze(output)

ω = minimum(result.omega_n)
u_vec(~, t) = 0.2 * [sin(2π * ω * t), sin(2π * ω * (t+0.5)), sin(2π * ω * (t+1))]

t1 = 0
t2 = 20
yy = ltisim(result.ss_eqns, u_vec, (t1, t2))

t = t1:(t2-t1)/1000:t2
y = hcat(yy.(t)...)'
f = hcat(u_vec.(0, t)...)'

xlabel = "Time [s]"
ylabel = "x [m], y [m], ψ [rad]"
label = ["x" "y" "ψ" "X" "Y" "N"]
lw = 2
size = (800, 400)

plots = [plot(t, [y f]; xlabel, ylabel, label, lw, size)]
summarize(system, result; plots)

# summarize(system, result; plots, format = :html)
# animate_modes(system, result)

#=
system = input_ex_hanging_chain()
sensors_animate!(system)
output = run_eom!(system)
result = analyze(output)

yy = ltisim(result.ss_eqns, u_vec, (t1, t2))
y = hcat(yy.(t)...)

animate_history(system, t, y)
=#




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

