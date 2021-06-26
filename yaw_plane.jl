
using EoM, Plots
plotly()

build_examples()
include(joinpath("examples", "input_ex_yaw_plane.jl"))

# Here you can enter your vehicle specs by name, including m, I, a, b ,cf, cr
# Make sure you add the property you want to set to the argument list of input_ex_yaw_plane after you set it.
# Properties you don't set will use defaults defined in input_ex_yaw_plane
m = 1500
a = 1.5
b = 1.6
temp(x) = input_ex_yaw_plane(;u = x, m, a, b)
# Here we set the speed, which get copied as vpts argument in run_eom, which in turn gets sent one at a time to the temp function, which finally sends them to the input_ex_yaw_plane function
vpts = 1:0.5:25
my_sys, my_eqns = run_eom(temp, :verbose; vpts)
my_result = analyze(my_eqns, :verbose)

function steer(t)
    delta = 0
    if t < 1
        delta = 0
    elseif t < 3
        delta = 2 * sin(pi * (t - 1))
    else
        delta = 0
    end
    delta
end

t = 0:0.05:20 # choose time interval
d = steer.(t)

# Note "my_result[end]", i.e., choose the fastest speed defined above as the choice for the time domain
z = splsim(my_result[end].ss_eqns, d, t)
res = hcat(z...) # merge vector of vectors into matrix

xlabel = "Time [s]"
lw = 2

# Plot yaw rate vs time
ylabel = "Yaw rate [deg/s], Steer angle [deg]"
label = ["Yaw rate" "Steer angle"]
p1 = plot(t, [res[1, :] d]; xlabel, ylabel, label, lw)

# plot body slip angle vs time
ylabel = "Body slip angle [deg], Steer angle [deg]"
label = ["Body slip angle" "Steer angle"]
p2 = plot(t, [res[2, :] d]; xlabel, ylabel, label, lw)

# plot understeer angle vs time
ylabel = "Understeer angle [deg], Steer angle [deg]"
label = ["Understeer angle" "Steer angle"]
p3 = plot(t, [res[3, :] d]; xlabel, ylabel, label, lw)

# plot lateral acceleration vs time
ylabel = "Lateral acceleration [g], Steer angle [deg]"
label = ["Lateral acceleration" "Steer angle"]
p4 = plot(t, [res[4, :] d]; xlabel, ylabel, label, lw)

# plot path
xlabel = "x [m]"
ylabel = "y [m]"
label = ""
p5 = plot(vpts[end] * t, res[5, :]; xlabel, ylabel, label, lw)

write_html(
    my_sys,
    my_result,
    p1,
    p2,
    p3,
    p4,
    p5,
    ss = [1, 2, 3, 4],
    bode = [2, 3],
    :verbose,
)

println("Done.")
