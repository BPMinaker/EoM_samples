
using EoM, Plots, ForwardDiff
plotly()

build_examples()
include(joinpath("examples", "input_ex_yaw_plane.jl"))

# Here you can enter your vehicle specs by name, including m, I, a, b, cf, cr
# Make sure you add the property you want to set to the argument list of input_ex_yaw_plane below after you set it.
# Properties you don't set will use defaults defined in input_ex_yaw_plane
m = 1500
a = 1.5
b = 1.6
# Define a dummy function temp that just calls our input function, but also adds the parameters we just set
temp(x) = input_ex_yaw_plane(;u = x, m, a, b)

# Here we set the speed in vpts, which gets passed as the vpts argument in run_eom, which in turn gets sent one at a time to the temp function, which finally sends them to the input_ex_yaw_plane function
vpts = 1:0.5:30
# Generate the equations of motion, but many times, for every different value of forward speed
my_sys, my_eqns = run_eom(temp, :verbose; vpts)
# Do the eigenvalues, freq resp, etc, for each forward speed
my_result = analyze(my_eqns, :verbose)

# Now, let's also do some time domain solutions
# Define the input function of the state (x) and time (t), but in this case, ignore the state
function steer(x, t)
    EoM.pulse(t,1,3) * 2 * sin(pi * (t - 1))
end

# Define time interval
t = 0:0.05:20

# Solve the equations of motion using the EoM sparse linear solver, with the input function we just defined
# We didn't send an inital condition, so the solver assumes all zeros
# Note we use "my_result[end]", i.e., choose the equations of motion for the fastest speed defined above as the choice for the time domain
y = splsim(my_result[end].ss_eqns, steer, t)
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

# plot lateral acceleration vs time
ylabel = "Lateral acceleration [g], Steer angle [deg]"
label = ["Lateral acceleration" "Steer angle"]
push!(plots, plot(t, [res[4, :] delta]; xlabel, ylabel, label, lw))

# plot path
xlabel = "x [m]"
ylabel = "y [m]"
label = ""
push!(plots, plot(vpts[end] * t, res[5, :]; xlabel, ylabel, label, lw))


# Now lets try some closed loop feedback, where the driver input depends on the location
# Define the road geometry.  Note that we assume small heading angles in the linear equations, so can't do circuits yet.
# A gentle slalom, 1 m amplitude with 100 ft (30.48 m spacing), here x is the x location
function track(x)
    # Define road y coordinate
    y(x) =  EoM.pulse(x,50,100) * (2 - 2*cos(2pi/100*(x-50))) + EoM.pulse(x, 100, 150) * 4 + EoM.pulse(x,150,200) * (2 + 2*cos(2pi/100*(x-150)))

#y(x)=sin(0.1x)

    # Use automatic differentiation to find the heading angle and curvature
    # As long as the angles are small we can approximate slope with the derivative and the curvature as the second derivative
    dy(x) = ForwardDiff.derivative(y,x) 
    d2y(x) = ForwardDiff.derivative(dy,x) 

    # Evaluate all three and return those values
    y(x),dy(x),d2y(x)
end

# Now let's define the driver model, based on the vehicle location and heading, and the road
function steer_driver(y, t)

    # Get vehicle location and heading from sensors (y is the output vector)
    offset = y[5]
    heading = y[6]

    # Get the road location, heading
    offset_t, heading_t, curvature = track(vpts[end]*t)

    # Find the error in location and heading
    offset_error = offset_t - offset
    heading_error = heading_t - heading

    # Compute the appropriate steer angle to return to the road
    180/pi*((a+b) * curvature + 1.1 * heading_error + 0.1 * offset_error)
end

# A dummy function to convert the driver model from a function of the output to a function of the state, because the solver requires the input to be a function of the state
steer2(x,t) = steer_driver(my_result[end].ss_eqns.C*x, t)

# Solve the equation of motion with the closed loop driver model
y = splsim(my_result[end].ss_eqns, steer2, t)
# Go back and figure out what steer angle the sriver model used, so we can plot it
delta = steer_driver.(y, t)
# Merge vector of vectors for plotting
res = hcat(y...)

# Plot yaw rate vs time
xlabel = "Time [s]"
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

# plot lateral acceleration vs time
ylabel = "Lateral acceleration [g], Steer angle [deg]"
label = ["Lateral acceleration" "Steer angle"]
push!(plots, plot(t, [res[4, :] delta]; xlabel, ylabel, label, lw))

# plot heading angle
ylabel = "Heading angle [deg], Track angle, Steer angle [deg]"
label = ["Heading angle" "Steer angle"]
push!(plots, plot(t, [180/pi*res[6, :] delta]; xlabel, ylabel, label, lw))

# plot path
xlabel = "x [m]"
ylabel = "y [m]"
label = ["Vehicle path" "Track"]
push!(plots, plot(vpts[end] * t, [res[5, :] first.(track.(vpts[end] * t))]; xlabel, ylabel, label, lw))

# Write all the stuff to the output
# Steady state plots of outputs 1 through 4, but Bode of only 2 and 3
ss = [1, 2, 3, 4]
bode = [2, 3]
write_html(my_sys, my_result, :verbose; plots, ss, bode)

println("Done.")
