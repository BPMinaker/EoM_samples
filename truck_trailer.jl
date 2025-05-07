module truck_trailer

using EoM

include(joinpath("models", "input_ex_truck_trailer.jl"))
# here you can enter your vehicle specs by name
m = 16975 / 9.81
Iz = 3508
d = 2.7
e = 2.9
h = 0.1
mt = 2000
It = 3000
vpts = 0.4:0.4:40

format = :screen
# format = :html

f(x) = input_ex_truck_trailer(; u=x, m, Iz, d, e, h, mt, It)
system = f.(vpts)
output = run_eom!.(system, vpts .== 1)
result = analyze.(output, vpts .== 1; freq=(-1, 1))

impulse = :skip
bode = :skip
#summarize(system, vpts, result; impulse)
summarize(system, vpts, result; impulse, bode, format)

# choose the equations of motion for 18 m/s (note function notation)
n = findfirst(vpts .== 18)
system = system[n]
result = result[n]
system.name *= " 18 m per s"

# equations are known, let's solve a time history
steer(t) = EoM.pulse(t, 1, 3) * 2 * sin(π * (t - 1))
u_vec(_, t) = [steer(t)] # define input function to be steer but to also accept x and then ignore it

# Define time interval
t1 = 0
t2 = 20

# solve the equations of motion
yoft = ltisim(result, u_vec, (t1, t2))

# sensors are, in order, r, β, α_u, ψ, a_lat

# plot yaw rate vs time
yidx = [1]
label, ylabel = ltilabels(system; yidx)
plots = [ltiplot(yoft; ylabel, label, yidx)]

# plot body slip angle vs time
yidx = [2]
label, ylabel = ltilabels(system; yidx)
push!(plots, ltiplot(yoft; ylabel, label, yidx))

# plot understeer angle vs time
yidx = [3]
label, ylabel = ltilabels(system; yidx)
push!(plots, ltiplot(yoft; ylabel, label, yidx))

# plot trailer sway angle vs time
yidx = [4]
label, ylabel = ltilabels(system; yidx)
push!(plots, ltiplot(yoft; ylabel, label, yidx))

# plot lateral acceleration vs time
yidx = [5]
label, ylabel = ltilabels(system; yidx)
push!(plots, ltiplot(yoft; ylabel, label, yidx))

impulse = :skip
ss = :skip
summarize(system, result; plots, impulse, ss, format)

# using EoM_X3D
# animate_modes(system, result)

end

println("Done.")
