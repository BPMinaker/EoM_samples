using EoM, Plots
plotly()

build_examples()
include(joinpath("examples", "input_ex_truck_trailer.jl"))
# here you can enter your vehicle specs by name
m = 16975 / 9.81
Iz = 3508
d = 2.7
e = 2.9
h = 0.1
mt = 2000
It = 3000
vpts = 1:0.5:35

f(x) = input_ex_truck_trailer(; u=x, m, Iz, d, e, h, mt, It)
system = f.(vpts)
output = run_eom!.(system, vpts .== 1)
result = analyze.(output, vpts .== 1)

# equations are known, let's solve a time history
# pulse function is 1 from t=1 to t=3, zero otherwise
# sin(pi * t) has a wavelength of 2 seconds (2 pi/ pi)
function steer(x, t)
    EoM.pulse(t, 1, 3) * 2 * sin(pi * (t - 1))
end

# Define time interval
t = 0:0.05:20
# solve the equations of motion using the EoM sparse linear solver, with the input function we just defined
# we didn't send an inital condition, so the solver assumes all zeros
# choose the equations of motion for 18 m/s (note function notation)
n = findfirst(vpts .== 18)

y = splsim(result[n].ss_eqns, steer, t)
# merge vector of vectors into matrix, so we can pull out individual outputs (rows) to plot
res = hcat(y...)
# evaluate the steer angle so we can include it in the plots 
delta = steer.(0, t)

xlabel = "Time [s]"
lw = 2 # thicker line weight

# plot yaw rate vs time
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

# plot trailer sway angle vs time
ylabel = "Sway angle [deg], Steer angle [deg]"
label = ["Sway angle" "Steer angle"]
push!(plots, plot(t, [res[4, :] delta]; xlabel, ylabel, label, lw))

# plot lateral acceleration vs time
ylabel = "Lateral acceleration [g], Steer angle [deg]"
label = ["Lateral acceleration" "Steer angle"]
push!(plots, plot(t, [res[5, :] delta]; xlabel, ylabel, label, lw))

bode = [0, 1, 1, 1, 0]
summarize(system, vpts, result, true; plots, bode)
# summarize(system, vpts, result, true; plots, bode, format = :html)

summarize(system[n], result[n], true; bode)

println("Done.")
