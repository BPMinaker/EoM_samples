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

f(x) = input_ex_truck_trailer(; u=x, m, Iz, d, e, h, mt, It)
system = f.(vpts)
output = run_eom!.(system, vpts .== 1)
result = analyze.(output, vpts .== 1; freq=(-1, 1))

impulse = :skip
summarize(system, vpts, result; impulse)
# summarize(system, vpts, result; impulse, format = :html)

# let's isolate one speed and expand
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
yoft = ltisim(result.ss_eqns, u_vec, (t1, t2))
t = t1:(t2-t1)/1000:t2
y = hcat(yoft.(t)...)'

r = y[:, 1]
β = y[:, 2]
α_u = y[:, 3]
γ = y[:, 4]
a_lat = y[:, 5]
δ = steer.(t) # evaluate the steer angle so we can include it in the plots 

xlabel = "Time [s]"
lw = 2 # thicker line weight
size = (800, 400)

# plot yaw rate vs time
ylabel = "Yaw rate [°/s], Steer angle [°]"
label = ["Yaw rate r" "Steer angle δ"]
plots = [plot(t, [r δ]; xlabel, ylabel, label, lw, size)]

# plot body slip angle vs time
ylabel = "Body slip angle [°], Steer angle [°]"
label = ["Body slip angle β" "Steer angle δ"]
push!(plots, plot(t, [β δ]; xlabel, ylabel, label, lw, size))

# plot understeer angle vs time
ylabel = "Understeer angle [°], Steer angle [°]"
label = ["Understeer angle" "Steer angle δ"]
push!(plots, plot(t, [α_u δ]; xlabel, ylabel, label, lw, size))

# plot trailer sway angle vs time
ylabel = "Sway angle [°], Steer angle [°]"
label = ["Sway angle γ" "Steer angle δ"]
push!(plots, plot(t, [γ δ]; xlabel, ylabel, label, lw, size))

# plot lateral acceleration vs time
ylabel = "Lateral acceleration [ge], Steer angle [°]"
label = ["Lateral acceleration" "Steer angle δ"]
push!(plots, plot(t, [a_lat δ]; xlabel, ylabel, label, lw, size))

impulse = :skip
summarize(system, result; plots, impulse)

#summarize(system, result; plots, format = :html)
#using EoM_X3D
#animate_modes(system, result)

end

println("Done.")
