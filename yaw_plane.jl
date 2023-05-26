using EoM, Plots
plotly()

include(joinpath("models", "input_ex_yaw_plane.jl"))

# here you can enter your vehicle specs by name, including m, Iz, a, b, cf, cr; make sure you add the property you want to set to the argument list of `input_ex_yaw_plane()` below after you set it; properties you don't set will use defaults defined in `input_ex_yaw_plane()`
m = 1600
fwf = 0.58
wb = 2.6

b = wb * fwf
a = wb - b
Iz = 2600
cf = 70000
cr = 80000

# define a dummy function that just calls our input function, but also adds the parameters we just set
f(x) = input_ex_yaw_plane(; u=x, m, a, b, Iz, cf, cr)

# here we set the speed in `vpts`, which gets sent one at a time to the `f()` function, which finally sends them to the `input_ex_yaw_plane()` function, where they determine the value of `u`
vpts = 1:0.5:30

# generate our system
system = f.(vpts)

# generate the equations of motion, but many times, for every different value of forward speed
# vpts .==1 is a vector of true/false, true on the first value, false everywhere else
# we use it to get a verbose output only on the first set of equations of motion
output = run_eom!.(system, vpts .== 1)

# do the eigenvalues, freq resp, etc, for each forward speed
result = analyze.(output, vpts .== 1)

# now, let's also do some time domain solutions; define the input function of the state (x) and time (t), but in this case, ignore the state
function u(x, t)
    EoM.pulse(t, 1, 3) * 2 * sin(π * (t - 1))
end

# define time interval
t = 0:0.05:20

# solve the equations of motion using the EoM sparse linear solver, with the input function we just defined; we didn't send an inital condition, so the solver assumes all zeros; note that `result` contains the equations of motion for every speed, but we pick out the set for 20 m/s)
# notation conflict here - u is default input to system, also forward speed, use `vel` for speed
vel = 20
n = findfirst(vpts .== vel)
y = splsim(result[n].ss_eqns, u, t)

# merge vector of vectors into matrix, so we can pull out individual outputs to plot, and re-evaluate the steer angle so we can include it in the plots 
res = hcat(y...)
δ = u.(0, t)

# Julia identifies every individual row of a matrix as a vector, so if we pull out just one row, it becomes a column
# another notation conflict, y is system output, but also lateral displacement, use `y_dist` for lateral displacement
r = res[1, :]
β = res[2, :]
α_u = res[3, :]
a_lat = res[4, :]
y_dist = res[5, :]
α_f = res[7, :]
α_r = res[8, :]

xlabel = "Time [s]"
lw = 2 # thicker line weight
size = (800, 600)

# plot yaw rate vs time
ylabel = "Yaw rate [deg/s], Steer angle [deg]"
label = ["Yaw rate r" "Steer angle δ"]
plots = [plot(t, [r δ]; xlabel, ylabel, label, lw, size)]

# plot body slip angle vs time
ylabel = "Body slip angle [deg], Steer angle [deg]"
label = ["Body slip angle β" "Steer angle δ"]
push!(plots, plot(t, [β δ]; xlabel, ylabel, label, lw, size))

# # plot understeer angle vs time
# ylabel = "Understeer angle [deg], Steer angle [deg]"
# label = ["Understeer angle α_u" "Steer angle δ"]
# push!(plots, plot(t, [α_u δ]; xlabel, ylabel, label, lw, size))

# plot slip angles vs time
ylabel = "Understeer angle [deg], Steer angle [deg]"
label = ["Slip angle α_f" "Slip angle α_r" "Understeer angle α_u" "Steer angle δ"]
push!(plots, plot(t, [α_f α_r α_u δ]; xlabel, ylabel, label, lw, size))


# plot lateral acceleration vs time
ylabel = "Lateral acceleration [g], Steer angle [deg]"
label = ["Lateral acceleration" "Steer angle δ"]
push!(plots, plot(t, [a_lat δ]; xlabel, ylabel, label, lw, size))

# plot path, noting that it is not even close to uniform scaling, x ~ 400 m, y ~ 2.5 m
xlabel = "x [m]"
ylabel = "y [m]"
label = ""
push!(plots, plot(vel * t, y_dist; xlabel, ylabel, label, lw, size))



# write all the results; steady state plots of outputs 1 through 4 (5 and 6 don't reach steady state), and Bode of only 2 and 3, as they are the only ones where input and output have the same units

ss = [1, 1, 1, 1, 0, 0, 1, 1]
bode = [0, 1, 1, 0, 0, 0, 1, 1]

summarize(system, vpts, result; plots, ss, bode)
# summarize(system, vpts, result; plots, ss, bode, format = :html)

println("Done.")
