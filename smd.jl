
# the first step is to load the `EoM` and other support libraries.
using EoM, Plots
plotly() # choose plotting engine

# we ask EoM to build a folder of example systems like the spring mass damper; you can build your own input files for other systems, but EoM has many examples; you should look into those files to see the structure of an EoM input file.

# the `:verbose` flag is optional, to get feedback on the progress of `build_examples()`
build_examples(:verbose)

# now, we load the function `input_ex_smd()`, which contains the definition of the spring mass damper system; we make Julia aware of the function by `including` the file that contains it.

# the `joinpath()` function inserts the appropriate separator, i.e., a forward slash or backslash, depending on the platform (Windows/Mac); you can `include` input files for systems you write yourself in the same way
include(joinpath("examples", "input_ex_smd.jl"))

# one of the cool features of Julia is that it allows `keyword arguments`, i.e., you can pass many arguments to a function, but you can include the name of the argument, so it doesn't matter if you mix up the order; the example input file allows you to redefine the values of the mass (m), the stiffness (k), and the damping (c); for example, you could call:

# input_ex_smd(m = 1.2)

# the drawback is that sometime you get cases like this:

# m = 1.2
# input_ex_smd(m=m)

# we can get around this weirdness using a semi-colon in the call to the function, if we want to send a local value to the same variable name in a function

# m=1.2
# input_ex_smd(;m)

# any values we don't supply just use defaults defined in the `input_ex_smd.jl` file.

# now, the twist is that we don't want to call the input function directly, but instead simply pass that function name onto a helper function called `run_eom()`, which will do some initial sorting and processing of the system definition, and then build the equations of motion; the `run_eom()` function returns the system definition, and the equations of motion; again, the `:verbose` flag is optional, to get feedback on the progress of `run_eom()`.

# so the structure could look like this:

# my_sys, my_eqns = run_eom(input_ex_smd, :verbose)

# the problem with this choice is that we can't pass the name of our input function to `run_eom()` without giving up the arguments that we want to send to `input_ex_smd()`, so instead, we create a dummy function `temp()`, that acts like a substitute for our input function; any calls to temp without any arguments will fill in our choices, and pass them to `input_ex_smd()`, so we could do

# temp() = input_ex_smd(k = 5, m = 1, c = 0.2)

# or

k = 5
m = 1
c = 0.2
temp() = input_ex_smd(;k, m, c)

# the advantage of the second choice is that now m, c, and k are available to use elsewhere in the code, if we wanted, and if we call `temp()` after any changes to m, c, or k, the updated values are used

# we send `temp()` to `run_eom()`; the optional `:verbose` argument tells `run_eom()` to print out progress to the screen
my_sys, my_eqns = run_eom(temp, :verbose)

# we can see what was in our system by looking in `my_sys`, but there are two wrinkles; first, my_sys is a actually vector, in case we want to automatically analyze a system with a range of parameters (more on that later), so we need to use [1] to get the first entry; second, we need to use the dot operator; in Julia, the dot `.` can be used in a structured variable, sort of like a container that holds other variables; the variable `my_sys[1]` is a structured variable; to see how many bodies there are, we can look at:
println(length(my_sys[1].bodys))

# there are two bodies, but one is the ground, which is added automatically, and we can see the names:
println(my_sys[1].bodys[1].name)
println(my_sys[1].bodys[2].name)

# you can find all the names of the attributes of a structured variable like so:
#println(fieldnames(EoM.mbd_system))
#println(fieldnames(body))

# the equations of motion are stored as a descriptor state space system (A,B,C,D,E) in `my_eqns`, which is also a vector; we can `print` or `display` them, but `display` looks much nicer
display(my_eqns[1])
display(my_eqns[1].A)

# if we want to know more about the behaviour of this system, we can do some analysis on the equations; again, `:verbose` is optional, and instructs `analyze()` to display progress messages
my_result = analyze(my_eqns, :verbose)

# the analysis converts the system to a standard state space form (A,B,C,D), where the E matrix is eliminated, the size of the A matrix is usually smaller after this happens, and almost certainly not the same as if we derived it by hand, even though it has the same properties

# just like `my_eqns` can hold multiple systems of equations, `my_result` can hold a result for each entry, and stores it in a vector
display(my_result[1].ss_eqns)
display(my_result[1].ss_eqns.A)

# we can also print the natural frequencies, the damping ratios, the time constants, and the wavelengths.
println("Eigenvalues")
display(my_result[1].e_val)
println("Natural frequencies")
display(my_result[1].omega_n)
println("Damping ratios")
display(my_result[1].zeta)
println("Time constants")
display(my_result[1].tau)
println("Wavelengths")
display(my_result[1].lambda)

# now, the ODE solver library in Julia is great, and has a number of very sophisticated algorithms, and can solve just about any type of ODE; however, in this case, our problem is quite a simple one, a linear ODE; for a linear ODE, it is often preferable to use a simpler ODE solver; there is one in the EoM library called `splsim()` (for sparse linear simulation); it uses something called `discrete time` to convert the differential equation to a difference equation, which it solves much faster

#when using `splsim`, we have to choose a set of uniform time points in advance, and provide a function that can compute the input at those time points; more small steps provide a more accurate solution, but take longer to do; splsim() will warn us if our time step is too big for the properties of the system

# define the time as a range, 0 to 20 seconds with steps of 0.02 seconds
t = 0:0.02:20

# define input forcing function; here we chose to excite the system near its natural frequency
w = 0.95 * my_result[1].omega_n[1]

# the input function we pass to `splsim()` has to be defined as a function of the state and time, i.e., it takes two arguments, but it doesn't actually have to use them both, in this case we only use a time dependent input
u(x,t) = sin(2pi * w * t)

# we pass the structured variable `ss_eqns` that holds the A, B, C, and D matrices, the input, and the time vector to EoM's own linear ODE solver; it solves the equation x_dot = Ax + Bu for x, then uses y = Cx + Du to solve for y, the output vector, which in this case has three entries: z, z_dot, and kz; we could include it initial state if we wanted but `splsim()` will just use zeroes if we don't specify anything else
y = splsim(my_result[1].ss_eqns, u, t)

# the return argument `y` is a vector of vectors, one output vector for each time point; Julia makes a distinction between a vector of vectors and a matrix; we can acess a vector of vectors like so: a[2][3] to the get third entry in the second vector, where a matrix is a[2,3] gives the entry in the second row, third column

# to read `y`, we use the `hcat()` function for horizontal concatenation (i.e., sticking columns together to form a matrix), and the `splatting` operator `...` to read all the entries from y; you can think of it is a short form for hcat(y[1], y[2] ... y[end]); the apostrophe `'` operator is a transpose, to write each output vector as a row instead of a column
res = [hcat(y...)' u.(0,t)]
# our result will have four columns: the displacement, the velocity, the spring force, and the applied force
# we get the applied force by attaching the result of input function, so we can plot them together; in this case we use the dot operator before the bracket `.(` to tell Julia that we are `vectorizing` the function, i.e., we are sending a vector of values to the function, and would like it to evaluate them one at a time, gather the results, and return them in a vector; vectorizing is computationally faster than using loops; in this case the 0 in the input is ignored, but the sin function is evaluated for each entry in the t vector

# we can make a plot; here we plot t on the x axis, and the first second, and fourth colums on the y axis (the displacement, velocity, and applied force, skipping the tension in the spring in the third column); this should show up in a tab in VSCode

println("Plotting...")

plots = [plot(t, res[:, [1, 2, 4]], lw = 2, xlabel = "Time [s]", label = ["z" "zdot" "f"])]
display(plots[1])
# the plot is stored in a vector called `plots`, and then sent to the screen


# let's reproduce the plot, but with the excitation frequency well below and well above the natural frequency; in both cases, the displacement should be smaller; `u()` is defined as a function of `w` so all we have to do is update `w`, and `u()` will update as well

w = 0.5 * my_result[1].omega_n[1]

y = splsim(my_result[1].ss_eqns, u, t)
res = [hcat(y...)' u.(0,t)]
push!(plots, plot(t, res[:, [1, 2, 4]], lw = 2, xlabel = "Time [s]", label = ["z" "zdot" "f"]))
# we can use the push!() function to add a new entry onto an existing vector

w = 2.0 * my_result[1].omega_n[1]

y = splsim(my_result[1].ss_eqns, u, t)
res = [hcat(y...)' u.(0,t)]
push!(plots, plot(t, res[:, [1, 2, 4]], lw = 2, xlabel = "Time [s]", label = ["z" "zdot" "f"]))

# to make a much more convenient result, we can send the system definition, the analysis results, and any extra plots to a helper function to make a nice summary; look in the `outputs` folder for a folder with today's date, and in that folder, a `Spring Mass Damper.html` file; you can change the folder name and filename with keyword arguments `folder` and `filename` if you really want; the default filename is taken from the model name in the input file; the data is also written to individual files as `output/date/filename/time/plot_1.html`, etc., which won't get overwritten if you run the analysis again (but the main html output file does!)

write_html(my_sys, my_result, :verbose; plots, bode = [3])
# make the Bode plot using the third ouput (kx) only, because the Bode plot should be dimensionless, i.e., input and output should have the same units, so we plot the ratio of spring force to applied force, as a function of frequency; the spring force is proportional to displacement, so we are really looking at displacement response

using EoM_X3D
# lastly we can use the EoM animation library to make animations of the mode shapes
animate_modes(my_sys[1], my_result[1], scale=0.2)

println("Done.")
