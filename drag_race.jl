using EoM, Interpolations, Plots
plotly() # choose plot engine, need plotly for html format plots

# call EoM function to build example input systems
build_examples()
# read in the function that will define our drag race vehicle model
include(joinpath("examples", "input_ex_drag_race.jl"))

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

# input engine specs
# engine torque [Nm] in evenly spaced engine speed increments from 0 to redline
# we need to fudge a little and have a torque value even at zero speed, otherwise no launch
ti = [30, 50, 75, 115, 150, 160, 150, 120]
wi = [0, 1000, 2000, 3000, 4000, 5000, 6000, 7000]
redline= wi[end]

# make the fit
te = LinearInterpolation(wi, ti)
# te is now a callable function, i.e., te(w) returns torque (in Nm) at engine speed w (in rpms)

# calculate shift speeds (at redline)
vmax = redline * 2pi / 60 * re / slip / fd ./ gear
println("Shift speeds [km/h]:")
display(round.(3.6 * vmax, digits = 2))

# suspension stiffness and damping - the default values should be close enough
cf = 4000 # front suspension damping [Ns/m]
cr = 4000 # rear damping
kf = 30000 # front suspension stiffness [N/m]
kr = 30000

# build the equations of motion
# define the function that builds our model, with our specs
model() = input_ex_drag_race(; re, a, b, m, h_G, kf, kr, cf, cr)

# pass the name of the that function to run_eom, which will build the equations of motion
my_sys, my_eqns = run_eom(model)
# do some analysis, to convert the equations to state space form, don't try to decompose
my_result = analyze(my_eqns, decomp = false)

# state space matrices
A = my_result[1].ss_eqns.A
B = my_result[1].ss_eqns.B
C = my_result[1].ss_eqns.C

# static preloads, also from our analysis
Zf0 = my_sys[1].flex_points[1].preload[1]
Zr0 = my_sys[1].flex_points[2].preload[1]
println("Static Zf=", round(Zf0, digits = 2), " N")
println("Static Zr=", round(Zr0, digits = 2), " N")

# build the function that is the input to the equation of motion
# we will call this function through the ODE solver
function u(x,t)
    # println("t ",t)
    # set empty inputs
    u = [0.0; 0.0]
    # u[1] aero resistance force
    # u[2] traction force
    # we will compute this, but need to define an empty vector first

    # find the outputs of model
    y = C * x
    # y[1] long position
    # y[2] long velocity
    # y[3] rear axle normal spring force
    # y[4] rear axle damper force

    # calculate air resistance from outputs
    rho = 1.23
    af = 2.85
    cd = 0.35
    u[1] = rho / 2 * af * cd * y[2] * abs(y[2]) # use speed times abs(speed) instead of speed squared to switch force direction if we are moving backwards

    # find which gear we should use
    # note the .< returns a vector, i.e, (y[2].<vmax) returns a vector of true or false where the current speed is below the shift point, we use findnext to find the index of the first true
    n = findnext(y[2] .< vmax, 1)
    if n === nothing # if we are above redline in top gear
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

    # calculate axle load,static plus dynamic load, note the dynamic is negative if the spring is in compression, but this should add to static, hence negative sign
    Zr = Zr0 - y[3] - y[4]

    # check for wheelspin, conditional
    X > mu * Zr && (X = mu * Zr)

    # include rolling resistance loss, note -= to subtract from current value, and sign function to reverse force if needed
    X -= (0.013 + 6.5e-6 * y[2]^2) * 9.81 * m * sign(y[2])

    # set traction force
    u[2] = X

    # return u
    u
end

## solve the ODE, and convert result to columns
xu = 0
t = 0:0.01:30
y, xu = splsim(my_result[1].ss_eqns, u, t, flag = true)
y = hcat(y...)'
ydot = (C * [A B] * xu)[2,:]

y[:, 2] *= 3.6 # scale second column to convert to km/h
y[:, 3] .+= y[:, 4] # add damping load to spring load, note .+= increments each entry in column 3 by the corresponding entry in column 4
y[:, 3] *= -1.0 # take compressive loads as positve (switch signs)
y[:, 3] .+= Zr0 # add static load, note .+= increments each entry in the vector
y[:, 3] /= 1000 # scale third column to get kN
ydot /= 9.81 # acc'n in g

# linear interpolation to find 1/4 mile, 60 mph times
# note 1/4 mile = 402.336 m, 60 mph = 96.5606 km/h
interp_ts = LinearInterpolation(y[:, 1], t)
interp_ut = LinearInterpolation(t, y[:, 2])
interp_tu = LinearInterpolation(y[:, 2], t)

# find quarter mile time and speed, round it, and print
if y[end, 1] > 402.336
    tf = interp_ts(402.336)
    println(
        "Quarter mile time: ",
        round(tf, digits = 2),
        " s at ",
        round(interp_ut(tf), digits = 1),
        " km/h.",
    )
else
    println("Simulation ended before 1/4 mile!  Increase the time interval.")
end

# find 0-60 time, round it, and print
if y[end, 2] > 96.5606
    println("0-60 mph time: ",round(interp_tu(96.5606), digits = 2), " s.")
else
    println("Simulation ended below 60 mph!")
end

lw = 2
xlabel = "Time [s]"
label = ""

ylabel = "Distance [m]"
plots = [plot(t, y[:, 1]; xlabel, ylabel, label, lw)]

ylabel = "Velocity [km/h]"
push!(plots,plot(t, y[:, 2]; xlabel, ylabel, label, lw))

ylabel = "Accl'n [g]"
push!(plots, plot(t, ydot; xlabel, ylabel, label, lw))

ylabel = "Z_r [kN]"
push!(plots, plot(t, y[:, 3]; ylims=(0,Inf), xlabel, ylabel, label, lw))

# pass all the results and plots to the html writer, skip the bode plots
write_html(my_sys, my_result, plots, bode = [])

using EoM_X3D
animate_modes(my_sys[1], my_result[1])

println("Done.")













# # define stopping condition (when distance travelled - 1/4 mile > 0)
# function dist_to_end(x, t, integrator)
#     y = C * x
#     y[1] - 402.336
# end

# # time to simulate
# tf = 30.0
# # choose the time interval
# tspan = (0.0, tf)
# # define initial positon and velocity, all zeros
# x0 = zeros(size(A, 1))
# # define and solve the ODE
# prob = ODEProblem(eqn_of_motion, x0, tspan)
# cb = ContinuousCallback(dist_to_end, terminate!, interp_points = 1000)
# x = solve(prob, Tsit5(), saveat = 0.01, callback = cb, progress = true)
# # the return argument x is actually a function that we can evaluate at any time but we choose to store it at 0.01 second intervals so we can treat it as data, in the structured variable x (x.u is a vector of vectors, one output vector every 0.01 seconds, time is in x.t)

# # back calculate the acceleration from the velocity
# # start by defining xdot as a vector of zeros, the same length as the size of the solution
# xdot = [zeros(size(x[1])) for i in x.u]
# # call the equation of motion, and fill in xdot, note the .( to call as a vector, using each entry in xdot, x.u, x.t
# eqn_of_motion.(xdot, x.u, 0, x.t)

# # take all those individual x.u vectors at each t and stack them into a matrix, using horizontal concatenation
# # the hcat function sticks things together, and the splatting operator ... takes all the vectors and makes them into a sequence of arguments, equivalent to writing out hcat(x.u[1],x.u[2],x.u[3],etc)
# # then multiply by C, to change states (x) to outputs (y) and transpose the result from rows to columns using '
# y = (C * hcat(x.u...))'
# # do the same for ydot, but only keep the 2nd column (acceleration), the first column is redunant, and last two aren't of interest
# ydot = (C*hcat(xdot...))[2, :]





