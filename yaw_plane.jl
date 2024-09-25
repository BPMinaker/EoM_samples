using EoM

include(joinpath("models", "input_ex_yaw_plane.jl"))

# here you can enter your vehicle specs by name, including m, Iz, a, b, cf, cr; make sure you add the property you want to set to the argument list of `input_ex_yaw_plane()` below after you set it; properties you don't set will use defaults defined in `input_ex_yaw_plane()`
m = 1600
fwf = 0.58 # b/(a+b)
wb = 2.6 # a+b

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
output = run_eom!.(system)

# do the eigenvalues, freq resp, etc, for each forward speed
result = analyze.(output; freq=(-1, 1))

# now, let's also do some time domain solutions; define the steer angle as a function of time
# a sin w dwell input ala FMVSS 126
# a 0.7 Hz sinewave with origin at t=2 times zero everywhere except times one from t=2 for 3/4 of a wavelength
# plus a constant negative one for 0.5 seconds,starting right after the 3/4 wavelength
# plus a 0.7 Hz sinewave with origin at t=2.5 times zero everywhere except times one for the last 1/4 of a wavelength
# all times 2
steer(t) = 2 * (
sin(2π * 0.7 * (t - 2)) * EoM.pulse(t, 2, 2 + 0.75 / 0.7)
- EoM.pulse(t, 2 + 0.75 / 0.7, 2.5 + 0.75 / 0.7)
+ sin(2π * 0.7 * (t - 2.5)) * EoM.pulse(t, 2.5 + 0.75 / 0.7, 2.5 + 1 / 0.7))

# define input function to be steer but to also accept x and then ignore it, then put it in a vector
u_vec(_, t) = [steer(t)]

# define time interval
t1 = 0
t2 = 20

# find which equations of motion are from when u=20, and use that to solve the time history
u = 20
n = findfirst(vpts .== u)
yoft = ltisim(result[n].ss_eqns, u_vec, (t1, t2))

# find 1000 sub-intervals
t = t1:(t2-t1)/1000:t2
y = hcat(yoft.(t)...)'
# merge vector of vectors into matrix, so we can pull out individual outputs to plot

# Julia identifies every individual row of a matrix as a vector, so if we pull out just one row, it becomes a column
# another notation conflict, y is system output, but also lateral displacement, use `y_dist` for lateral displacement
r = y[:, 1]
β = y[:, 2]
α_u = y[:, 3]
a_lat = y[:, 4]
y_dist = y[:, 5]
α_f = y[:, 7]
α_r = y[:, 8]
δ = steer.(t)
# re-evaluate the steer angle so we can include it in the plots

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
impulse = :skip

summarize(system, vpts, result; plots, ss, impulse)
# summarize(system, vpts, result; plots, ss, impulse, format = :html)

println("Done.")
