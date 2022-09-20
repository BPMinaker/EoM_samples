using EoM, Interpolations, Plots
plotly() # choose plot engine

# call EoM function to build example input systems
build_examples()
# read in the function that will define our drag race vehicle model
include(joinpath("examples", "input_ex_drag_race.jl"))

# set the values of the parameters here
# tire radius in meters, for example, for 195/50-R16
re = (8 * 25.4 + 0.5 * 195) / 1000
eta = 0.9 # driveline efficiency
gear = [5, 4, 3, 2, 1] # ratios, starting from 1st
fd = 4.1 # final drive ratio
a = 1.2 # cg location [m]
b = 1.4

m = 1500 # mass
h_G = 0.4 # height of centre of mass [m]
mu = 0.9 # friction limit
# assume 5% tire slip, for purpose of computing shift speeds
slip = 1.05
rdf = 1.0 # rear drive fraction

# input engine specs
# engine torque [Nm] in evenly spaced engine speed increments from 0 to redline
# we need to fudge a little and have a torque value even at zero speed, otherwise no launch
ti = [30, 50, 75, 115, 150, 160, 150, 120]
wi = [0, 1000, 2000, 3000, 4000, 5000, 6000, 7000]
redline = wi[end]

# make the fit, linear is not the best, but good enough
te = LinearInterpolation(wi, ti)
# te is now a callable function, i.e., te(w) returns torque (in Nm) at engine speed w (in rpms)

# calculate shift speeds (at redline)
vmax = redline * 2pi / 60 * re / slip / fd ./ gear
println("Shift speeds [km/h]:")
display(round.(3.6 * vmax, digits=2))

# suspension stiffness and damping - the default values should be close enough
cf = 4000 # front suspension damping [Ns/m]
cr = 4000 # rear damping
kf = 30000 # front suspension stiffness [N/m]
kr = 30000

# build the equations of motion
# define the function that builds our model, with our specs
system = input_ex_drag_race(; re, a, b, m, h_G, kf, kr, cf, cr)

# pass the name of the that function to run_eom, which will build the equations of motion
output = run_eom!(system)

# static preloads are computed as part of the generation of the equations of motion
# we need these for the tire model, so save them in convenient notation
Zf0 = system.flex_points[1].preload[1]
Zr0 = system.flex_points[2].preload[1]
println("Static Zf=", round(Zf0, digits=2), " N")
println("Static Zr=", round(Zr0, digits=2), " N")

# do some analysis, to convert the equations to state space form (don't try to decompose because too many repeated zero eigenvalues makes the algorithm fail)
result = analyze(output)

# save the state space matrices in a convenient notation
A = result.ss_eqns.A
B = result.ss_eqns.B
C = result.ss_eqns.C

# build the function that is the input to the equation of motion (i.e., the external forces)
# we will call this function through the ODE solver
function u(x, t)
    # println("t ",t)
    # define the vector and set 0 values as placeholders
    # we will call this u_vec instead of u to keep it distinct from the function name, even though Julia doesn't care
    u_vec = [0.0; 0.0]
    # u_vec[1] aero resistance force
    # u_vec[2] traction force
    # note that these are defined in the input definition file
    # we will compute them and store them in u_vec

    # find the outputs of the model
    # these are also defined in the input definition file
    y = C * x
    # y[1] long position
    # y[2] long velocity
    # y[3] long accl'n (note that we we can't use this to find u_vec because it depends directly on u_vec!)
    # y[4] rear axle normal spring force
    # y[5] rear axle damper force
    # y[6] front axle normal spring force
    # y[7] front axle damper force


    # calculate air resistance from outputs
    rho = 1.23
    af = 2.85
    cd = 0.35
    u_vec[1] = rho / 2 * af * cd * y[2] * abs(y[2]) # use speed times abs(speed) instead of speed squared to switch force direction if we are moving backwards (which should never happen, but sometimes the rolling resistance model can cause weird things at very low speed)

    # find which gear we should use
    # note the .< returns a vector, i.e, (y[2] .< vmax) returns a vector of true or false where the current speed is below the shift point, we use findnext to find the index of the first true
    n = findnext(y[2] .< vmax, 1)
    if isnothing(n) # if we are above redline in top gear
        n = length(gear) # use top gear
    end

    # find engine speed (in rpm)
    w = y[2] * gear[n] * fd * slip * 60 / 2pi / re

    # if somehow we are moving backwards, set w=0, to prevent interpolation error (although it means we must have an issue somewhere else!)
    # note the shortcut conditional, in lieu of if statement
    w < 0 && (w = 0)

    # rev limiter, again shortcut conditional
    w > redline && (w = redline)

    # calculate traction force (axle torque/radius)
    X = te(w) * gear[n] * fd * eta / re
    Xr = rdf * X
    Xf = (1 - rdf) * X

    # calculate axle load, static plus dynamic load, note the dynamic is negative if the spring is in compression, but this should add to static, hence negative sign
    Zr = Zr0 - y[4] - y[5]
    Zf = Zf0 - y[6] - y[7]

    # check for wheelspin, conditional
    if Xr > mu * Zr
        Xr = mu * Zr
    end
    if Xf > mu * Zf
        Xf = mu * Zf
    end

    if Xf > (1 - rdf) / rdf * Xr
        Xf = (1 - rdf) / rdf * Xr
    end
    if Xr > rdf / (1 - rdf) * Xf
        Xr = rdf / (1 - rdf) * Xf
    end

    # include rolling resistance loss, and note sign function to reverse force if needed, set traction force
    u_vec[2] = Xr + Xf - (0.013 + 6.5e-6 * y[2]^2) * 9.81 * m * sign(y[2])

    # return u_vec
    u_vec
end

# solve the ODE
# choose the time interval and step size
println("Solving time history...")
t = 0:0.01:30
# pass the state space matrices, the input function, and the time interval to the solver
# it will assume zeros as inital conditions
y = splsim(result.ss_eqns, u, t)
# convert the vector of vectors to a matrix, and transpose it so the outputs are in columns for plotting
y = hcat(y...)'

y[:, 2] *= 3.6 # scale second column to convert to km/h
y[:, 3] /= 9.81 # acc'n in g
y[:, 4] .+= y[:, 5] # add damping load to spring load, note .+= increments each entry in column 3 by the corresponding entry in column 4
y[:, 4] *= -1.0 # take compressive loads as positve (switch signs)
y[:, 4] .+= Zr0 # add static load, note .+= increments each entry in the vector
y[:, 4] /= 1000 # scale third column to get kN
y[:, 6] .+= y[:, 7]
y[:, 6] *= -1.0 # take compressive loads as positve (switch signs)
y[:, 6] .+= Zf0 # add static load, note .+= increments each entry in the vector
y[:, 6] /= 1000 # scale third column to get kN


# linear interpolation to find 1/4 mile, 60 mph times
# note 1/4 mile = 402.336 m, 60 mph = 96.5606 km/h
interp_ts = LinearInterpolation(y[:, 1], t)
interp_ut = LinearInterpolation(t, y[:, 2])
interp_tu = LinearInterpolation(y[:, 2], t)

# find quarter mile time and speed, round it, and print
if y[end, 1] > 402.336
    tf = interp_ts(402.336)
    println("Quarter mile time: ", round(tf, digits=2), " s at ", round(interp_ut(tf), digits=1), " km/h.")
else
    println("Simulation ended before 1/4 mile!  Increase the time interval.")
end

# find 0-60 time, round it, and print
if y[end, 2] > 96.5606
    println("0-60 mph time: ", round(interp_tu(96.5606), digits=2), " s.")
else
    println("Simulation ended below 60 mph!")
end

# reduce the number of points that are plotted
t = t[1:5:end]
y = y[1:5:end, :]

# set plot text, etc
lw = 2 # thicker plot lineweight
xlabel = "Time [s]"
label = ""
ylabel = "Distance [m]"
# make the first plot and save it in a vector
plots = [plot(t, y[:, 1]; xlabel, ylabel, label, lw)]

# update label, make the next plot, and push it onto the plot vector
ylabel = "Velocity [km/h]"
push!(plots, plot(t, y[:, 2]; xlabel, ylabel, label, lw))

ylabel = "Accl'n [g]"
push!(plots, plot(t, y[:, 3]; ylims=(0, Inf), xlabel, ylabel, label, lw))

label = ["Z_r [kN]" "Z_f [kN]"]
ylabel = "Axle vertical load [N]"
push!(plots, plot(t, y[:, [4, 6]]; ylims=(0, Inf), xlabel, ylabel, label, lw))

# pass all the results and plots, skip the Bode plots for now
summarize(system, result; plots, bode=[])

# using DelimitedFiles
# writedlm(joinpath("output","data.txt"), [t y])

println("Done.")
