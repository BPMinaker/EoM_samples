
using EoM, Plots
plotly()

build_examples()
include(joinpath("examples", "input_ex_truck_trailer.jl"))
# here you can enter your vehicle specs by name
m = 16975 / 9.81
I = 3508
d = 2.7
e = 2.9
h = 0.1
temp(x) = input_ex_truck_trailer(;u = x, m, I, d, e, h)
#speed = 20
vpts = 1:0.5:35
my_sys, my_eqns = run_eom(temp, :verbose; vpts)
my_result = analyze(my_eqns, :verbose)


function steer(x, t)
    EoM.pulse(t,1,3) * 2 * sin(pi * (t - 1))
end

# Define time interval
t = 0:0.05:20

# Solve the equations of motion using the EoM sparse linear solver, with the input function we just defined
# We didn't send an inital condition, so the solver assumes all zeros
# Note we use "my_result[35]", i.e., choose the equations of motion for 18 m/s
y = splsim(my_result[35].ss_eqns, steer, t)
# Merge vector of vectors into matrix, so we can pull out individual outputs (rows) to plot
res = hcat(y...)
# Evaluate the steer angle so we can include it in the plots 
delta = steer.(0,t)

xlabel = "Time [s]"
lw = 2 # thicker line weight

# Plot yaw rate vs time
ylabel = "Yaw rate [deg/s], Steer angle [deg]"
label = ["Yaw rate" "Steer angle"]
plots = [plot(t, [res[1, :] delta]; xlabel, ylabel, label, lw)]

# plot body slip angle vs time
ylabel = "Body slip angle [deg], Steer angle [deg]"
label = ["Body slip angle" "Steer angle"]
push!(plots, plot(t, [res[2, :] delta]; xlabel, ylabel, label, lw))

# plot understeer angle vs time
ylabel = "Understeer angle [deg], Steer angle [deg]"
label = ["Understeer angle" "Steer angle"]
push!(plots, plot(t, [res[3, :] delta]; xlabel, ylabel, label, lw))

# plot understeer angle vs time
ylabel = "Sway angle [deg], Steer angle [deg]"
label = ["Sway angle" "Steer angle"]
push!(plots, plot(t, [res[4, :] delta]; xlabel, ylabel, label, lw))

# plot lateral acceleration vs time
ylabel = "Lateral acceleration [g], Steer angle [deg]"
label = ["Lateral acceleration" "Steer angle"]
push!(plots, plot(t, [res[5, :] delta]; xlabel, ylabel, label, lw))


# Now lets repeat it at 20 m/s

y = splsim(my_result[39].ss_eqns, steer, t)
# Merge vector of vectors into matrix, so we can pull out individual outputs (rows) to plot
res = hcat(y...)
# Evaluate the steer angle so we can include it in the plots 
delta = steer.(0,t)


# Plot yaw rate vs time
ylabel = "Yaw rate [deg/s], Steer angle [deg]"
label = ["Yaw rate" "Steer angle"]
push!(plots, plot(t, [res[1, :] delta]; xlabel, ylabel, label, lw))

# plot body slip angle vs time
ylabel = "Body slip angle [deg], Steer angle [deg]"
label = ["Body slip angle" "Steer angle"]
push!(plots, plot(t, [res[2, :] delta]; xlabel, ylabel, label, lw))

# plot understeer angle vs time
ylabel = "Understeer angle [deg], Steer angle [deg]"
label = ["Understeer angle" "Steer angle"]
push!(plots, plot(t, [res[3, :] delta]; xlabel, ylabel, label, lw))

# plot understeer angle vs time
ylabel = "Sway angle [deg], Steer angle [deg]"
label = ["Sway angle" "Steer angle"]
push!(plots, plot(t, [res[4, :] delta]; xlabel, ylabel, label, lw))

# plot lateral acceleration vs time
ylabel = "Lateral acceleration [g], Steer angle [deg]"
label = ["Lateral acceleration" "Steer angle"]
push!(plots, plot(t, [res[5, :] delta]; xlabel, ylabel, label, lw))

write_html(my_sys, my_result, :verbose; plots, bode = [2, 3, 4])

println("Done.")
