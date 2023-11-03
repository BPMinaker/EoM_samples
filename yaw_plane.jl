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
vpts = 0.4:0.4:40

# generate our system
system = f.(vpts)

# generate the equations of motion, but many times, for every different value of forward speed
# vpts .==1 is a vector of true/false, true on the first value, false everywhere else
# we use it to get a verbose output only on the first set of equations of motion
output = run_eom!.(system, vpts .== 1)

# do the eigenvalues, freq resp, etc, for each forward speed
result = analyze.(output, vpts .== 1)

# now, let's also do some time domain solutions; define the steer angle as a function of time
# pulse function is 1 from t=1 to t=3, 0 otherwise
# sin(pi * t) has a wavelength of 2 seconds (2 pi/ pi), same as pulse
steer(t) = EoM.pulse(t, 1, 3) * 2 * sin(π * (t - 1))
input(~, t) = steer(t) # define input function to be steer but to also accept x and then ignore it

# define time interval
t = 0:0.05:20

# solve the equations of motion using the EoM sparse linear solver, with the input function we just defined; we didn't send an inital condition, so the solver assumes all zeros; note that `result` contains the equations of motion for every speed, but we pick out the set for 20 m/s)

u = 20
n = findfirst(vpts .== u)
y = splsim(result[n].ss_eqns, input, t)

# merge vector of vectors into matrix, so we can pull out individual outputs to plot, and re-evaluate the steer angle so we can include it in the plots 
res = hcat(y...)

# Julia identifies every individual row of a matrix as a vector, so if we pull out just one row, it becomes a column
# another notation conflict, y is system output, but also lateral displacement, use `y_dist` for lateral displacement
r = res[1, :]
β = res[2, :]
α_u = res[3, :]
a_lat = res[4, :]
y_dist = res[5, :]
α_f = res[7, :]
α_r = res[8, :]
δ = steer.(t)

xlabel = "Time [s]"
lw = 2 # thicker line weight
size = (800, 400)

# plot yaw rate vs time
ylabel = "Yaw rate [° s^-1], Steer angle [°]"
label = ["Yaw rate r" "Steer angle δ"]
plots = [plot(t, [r δ]; xlabel, ylabel, label, lw, size)]

# plot body slip angle vs time
ylabel = "Body slip angle [°], Steer angle [°]"
label = ["Body slip angle β" "Steer angle δ"]
push!(plots, plot(t, [β δ]; xlabel, ylabel, label, lw, size))

# plot slip angles vs time
ylabel = "Understeer angle [°], Steer angle [°]"
label = ["Slip angle α_f" "Slip angle α_r" "Understeer angle α_u" "Steer angle δ"]
push!(plots, plot(t, [α_f α_r α_u δ]; xlabel, ylabel, label, lw, size))

# plot lateral acceleration vs time
ylabel = "Lateral acceleration [ge], Steer angle [°]"
label = ["Lateral acceleration" "Steer angle δ"]
push!(plots, plot(t, [a_lat δ]; xlabel, ylabel, label, lw, size))

# plot path, noting that it is not even close to uniform scaling, x ~ 400 m, y ~ 2.5 m
xlabel = "x [m]"
ylabel = "y [m]"
label = ""
push!(plots, plot(u * t, y_dist; xlabel, ylabel, label, lw, size))



# write all the results; steady state plots of outputs 1 through 4, 7, 8 (5 and 6 don't reach steady state)

ss = [1, 1, 1, 1, 0, 0, 1, 1]

summarize(system, vpts, result; plots, ss)
summarize(system, vpts, result; plots, ss, format = :html)

println("Done.")
