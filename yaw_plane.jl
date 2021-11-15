using EoM, Plots
plotly()

build_examples()
include(joinpath("examples", "input_ex_yaw_plane.jl"))

# here you can enter your vehicle specs by name, including m, Iz, a, b, cf, cr; make sure you add the property you want to set to the argument list of `input_ex_yaw_plane()` below after you set it; properties you don't set will use defaults defined in `input_ex_yaw_plane()`

m = 1500
a = 1.5
b = 1.6

# define a dummy function that just calls our input function, but also adds the parameters we just set

f(x) = input_ex_yaw_plane(; u = x, m, a, b)

# here we set the speed in `vpts`, which gets sent one at a time to the `temp()` function, which finally sends them to the `input_ex_yaw_plane()` function, where they determine the value of `u`

vpts = 1:0.5:30

# generate our system

system = f.(vpts)

# generate the equations of motion, but many times, for every different value of forward speed

output = run_eom!.(system, vpts .== 1)

# do the eigenvalues, freq resp, etc, for each forward speed

result = analyze(output, true)

# now, let's also do some time domain solutions; define the input function of the state (x) and time (t), but in this case, ignore the state

function steer(x, t)
    EoM.pulse(t, 1, 3) * 2 * sin(pi * (t - 1))
end

# define time interval

t = 0:0.05:20

# solve the equations of motion using the EoM sparse linear solver, with the input function we just defined; we didn't send an inital condition, so the solver assumes all zeros; note that `result` contains the equations of motion for every speed, but we pick out the set for 20 m/s, as follows (vpt[39] = 20)

n = findfirst(vpts .== 20)

y = splsim(result(n).ss_eqns, steer, t)

# merge vector of vectors into matrix, so we can pull out individual outputs to plot, and evaluate the steer angle so we can include it in the plots 
res = hcat(y...)

# Julia identifies every individual row of a matrix as a vector, so if we pull out just one row, it becomes a column

r = res[1, :]
β = res[2, :]
α_u = res[3, :]
a_lat = res[4, :]
δ = steer.(0, t)

xlabel = "Time [s]"
lw = 2 # thicker line weight

# plot yaw rate vs time
ylabel = "Yaw rate [deg/s], Steer angle [deg]"
label = ["Yaw rate" "Steer angle"]
plots = [plot(t, [r δ]; xlabel, ylabel, label, lw)]

# plot body slip angle vs time
ylabel = "Body slip angle [deg], Steer angle [deg]"
label = ["Body slip angle" "Steer angle"]
push!(plots, plot(t, [β δ]; xlabel, ylabel, label, lw))

# plot understeer angle vs time
ylabel = "Understeer angle [deg], Steer angle [deg]"
label = ["Understeer angle" "Steer angle"]
push!(plots, plot(t, [α_u δ]; xlabel, ylabel, label, lw))

# plot lateral acceleration vs time
ylabel = "Lateral acceleration [g], Steer angle [deg]"
label = ["Lateral acceleration" "Steer angle"]
push!(plots, plot(t, [a_lat δ]; xlabel, ylabel, label, lw))

# plot path
xlabel = "x [m]"
ylabel = "y [m]"
label = ""
push!(plots, plot(vpts[end] * t, res[5, :]; xlabel, ylabel, label, lw))

# write all the stuff to the output; steady state plots of outputs 1 through 4, but Bode of only 2 and 3

ss = [1, 2, 3, 4]
bode = [2, 3]

summarize(system, vpts, result; plots, ss, bode)

#write_html(system, vpts, result, verbose; plots, ss, bode)

println("Done.")
