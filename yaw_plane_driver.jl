using EoM, Plots, ForwardDiff
plotly()

build_examples()
include(joinpath("examples", "input_ex_yaw_plane.jl"))

# here you can enter your vehicle specs by name, including m, I, a, b, cf, cr; make sure you add the property you want to set to the argument list of input_ex_yaw_plane below after you set it; properties you don't set will use defaults defined in `input_ex_yaw_plane()`

m = 1500
a = 1.5
b = 1.6

# define a dummy function that just calls our input function, but also adds the parameters we just set

u = 30
system = input_ex_yaw_plane(; u, m, a, b)

# here we set the speed in vpts, which gets passed as the vpts argument in run_eom, which in turn gets sent one at a time to the temp function, which finally sends them to the input_ex_yaw_plane function

# generate the equations of motion

output = run_eom!(system, true)

# do the eigenvalues, freq resp

result = analyze(output, true)

# define time interval

t = 0:0.05:20

# now lets try some closed loop feedback, where the driver input depends on the location; define the road geometry; note that we assume small heading angles in the linear equations, so can't do circuits yet

function track(x)

    # define road y coordinate using the built-in function pulse to paste together a piecewise function of sines and constant

    y(x) = EoM.pulse(x, 50, 100) * (2 - 2 * cos(2pi / 100 * (x - 50))) + EoM.pulse(x, 100, 150) * 4 + EoM.pulse(x, 150, 200) * (2 + 2 * cos(2pi / 100 * (x - 150)))

    # use automatic differentiation to find the heading angle and curvature; as long as the angles are small we can approximate slope with the derivative and the curvature as the second derivative; automatic differentiation is a powerful numerical technique to compute the derivative of any function, using the fact that every function must be computed using basic arithmetic operations

    dy(x) = ForwardDiff.derivative(y, x)
    d2y(x) = ForwardDiff.derivative(dy, x)

    # evaluate all three and return those values
    y(x), dy(x), d2y(x)
end

# now let's define the driver model, based on the vehicle location and heading, and the road

function steer_driver(y, t)

    # get vehicle location and heading from sensors (y is the output vector)
    offset = y[5]
    heading = y[6] * pi/180 # convert back to radians

    # get the road location, heading
    offset_t, heading_t, curvature = track(u * t)

    # find the error in location and heading
    offset_error = offset_t - offset
    heading_error = heading_t - heading

    # compute the appropriate steer angle to return to the road
    180 / pi * ((a + b) * curvature + 1.1 * heading_error + 0.1 * offset_error)
end

# define a dummy function to convert the driver model from a function of the output to a function of the state, because the solver requires the input to be a function of the state

steer(x, t) = steer_driver(result.ss_eqns[1].C * x, t)

# solve the equation of motion with the closed loop driver model

y = splsim(result.ss_eqns, steer, t)

# go back and figure out what steer angle the sriver model used, so we can plot it

#delta = steer_driver.(y, t)

# merge vector of vectors for plotting
res = [hcat(y...)' steer_driver.(y, t)]

# Plot yaw rate vs time
xlabel = "Time [s]"
ylabel = "Yaw rate [deg/s], Steer angle [deg]"
label = ["Yaw rate" "Steer angle"]
lw = 2 # thicker line weight
plots = [plot(t, res[:, [1, end]]; xlabel, ylabel, label, lw)]

# plot body slip angle vs time
ylabel = "Body slip angle [deg], Steer angle [deg]"
label = ["Body slip angle" "Steer angle"]
push!(plots, plot(t, res[:, [2, end]]; xlabel, ylabel, label, lw))

# plot understeer angle vs time
ylabel = "Understeer angle [deg], Steer angle [deg]"
label = ["Understeer angle" "Steer angle"]
push!(plots, plot(t, res[:, [3, end]]; xlabel, ylabel, label, lw))

# plot lateral acceleration vs time
ylabel = "Lateral acceleration [g], Steer angle [deg]"
label = ["Lateral acceleration" "Steer angle"]
push!(plots, plot(t, res[:, [4, end]]; xlabel, ylabel, label, lw))

# plot heading angle
ylabel = "Heading angle [deg], Track angle, Steer angle [deg]"
label = ["Heading angle" "Steer angle"]
push!(plots, plot(t, [180 / pi * res[:, 6] res[:, end]]; xlabel, ylabel, label, lw))

# plot path
xlabel = "x [m]"
ylabel = "y [m]"
label = ["Vehicle path" "Track"]
push!(plots, plot(u * t, [res[:, 5] first.(track.(u * t))]; xlabel, ylabel, label, lw))

# write all the stuff to the output; skip steady state, Bode plots
ss = []
bode = []
summarize(system, result, false; plots, ss, bode)

println("Done.")
