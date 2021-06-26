
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

# input_ex_smd(m=1.2)

# the drawback is that sometime you get cases like this:

# m=1.2
# input_ex_smd(m=m)

# we can get around this weirdness using a semi-colon in the call to the function, if we want to send a local value to the same variable name in a function

# m=1.2
# input_ex_smd(;m)

# any values we don't supply just use defaults defined in the `input_ex_smd.jl` file.

# now, the twist is that we don't want to call the input function directly, but instead simply pass that function name onto a helper function called `run_eom()`, which will do some initial sorting and processing of the system definition, and then build the equations of motion; the `run_eom()` function returns the system definition, and the equations of motion; again, the `:verbose` flag is optional, to get feedback on the progress of `run_eom()`.

# so the structure could look like this:

# my_sys, my_eqns=run_eom(input_ex_smd, :verbose)

# the problem with this choice is that we can't pass the name of our input function to `run_eom()` without giving up the arguments that we want to send to `input_ex_smd()`, so instead, we create a function handle `input_hdl`, that acts like a substitute for our function, and send that to `run_eom()` instead
input_hdl() = input_ex_smd(k = 5, m = 1, c = 0.2)
my_sys, my_eqns = run_eom(input_hdl, :verbose)

# we can see what was in our system by looking in `my_sys`, but there are two wrinkles; first, my_sys is a vector, in case we want to automatically analyze a system with a range of parameters, so we need to use [1] to get the first entry; second we need to use the dot operator; in Julia, the dot `.` can be used in a structured variable, sort of like a container that holds other variables; the variable `my_sys[1]` is a structured variable; to see how many bodies there are, we can look:
println(length(my_sys[1].bodys))

# there are two bodies, but one is the ground, which is added automatically, and we can see the names:
println(my_sys[1].bodys[1].name)
println(my_sys[1].bodys[2].name)

# the equations of motion are stored as a descriptor state space system in `my_eqns`, which is also a vector; we can `print` or `display` them, but `display` looks much nicer
display(my_eqns[1])

display(my_eqns[1].A)

# if we want to know more about the behaviour of this system, we can do some analysis on the equations
my_result = analyze(my_eqns, :verbose)

# the analysis converts the system to a standard state space form, where the E matrix is eliminated, the size of the A matrix is usually smaller after this happens, and almost certainly not the same as if we derived it by hand, even though it has the same properties

# just like `my_eqns` can hold multiple system of equations, `my_result` can hold a result for each entry, and stores it in a vector
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

# now, the ODE solver library in Julia is great, and has a number of very sophisticated algorithms, and can solve just about any type of ODE; however, in this case, our problem is quite a simple one, a linear ODE; for a linear ODE, it is often preferable to use a simpler ODE solver; there is one in the EoM library called `splsim()` (for sparse linear simulation); when using `splsim`, we have to choose a set of uniform time points in advance, and compute the input at those time points

# define the time
t = 0:0.02:20

# define input forcing function; here we chose to excite the system near its natural frequency
w = 0.95 * my_result[1].omega_n[1]

# the other use of the dot operator in Julia is to vectorize operations; note the `sin.(  )`, as `t` is a vector, and we want to evaluate the sin function at each point in `t`
u=sin.(2pi * w * t)

# we pass the structured variable `ss_eqns` that holds the A, B, C, and D matrices, the input, and the time vector to EoM's own linear ODE solver; it solves the equation x_dot = Ax + Bu for x, then uses y = Cx +Du to solve for y, the output vector, which in this case has three entries: z, z_dot, and kz
y = splsim(my_result[1].ss_eqns, u, t)

# the return argument `y` is a vector of vectors, one output vector for each time point; Julia makes a disrinction between a vector of vectors and a matrix; to read it, we use the `hcat()` function for horizontal concatenation (i.e., sticking columns together to form a matrix), and the splatting operator `...` to read all the entries from y; you can think of it is a short form for hcat(y[1], y[2] ... y[end]); the `'` operator is a transpose, to write each output vector as a row instead of a column
res = [hcat(y...)' hcat(u...)']
# result will have four columns: the displacement, the velocity, the spring force, and the applied force

# we can make a plot; here we plot t on the x axis, and the first second, and fourth colums on the y axis (the displacement, velocity, and applied force, skipping the tension in the spring in the third column); this should show up in the tab

println("Plotting...")
display(plot(t, res[:, [1, 2, 4]], lw = 2, xlabel = "Time [s]", label = ["z" "zdot" "f"]))

# change plot engine so we can save the plots as html
plotly()

p1 = plot(t, res[:, [1, 2, 4]], lw = 2, xlabel = "Time [s]", label = ["z" "zdot" "f"])

# let's reproduce the plot, but with the excitation frequency well below and well above the natural frequency; in both cases, the displacement should be smaller

w = 0.5 * my_result[1].omega_n[1]
u=sin.(2pi * w * t)

y = splsim(my_result[1].ss_eqns, u, t)
res = [hcat(y...)' hcat(u...)']
p2 = plot(t, res[:, [1, 2, 4]], lw = 2, xlabel = "Time [s]", label = ["z" "zdot" "f"])

w = 2.0 * my_result[1].omega_n[1]
u=sin.(2pi * w * t)

y = splsim(my_result[1].ss_eqns, u, t)
res = [hcat(y...)' hcat(u...)']
p3 = plot(t, res[:, [1, 2, 4]], lw = 2, xlabel = "Time [s]", label = ["z" "zdot" "f"])

# to make a much more convenient result, we can send the system definition, the analysis results, and any extra plots to a helper function to make a nice summary; look in the `outputs` folder for a folder with today's date, and in that folder a `result.html` file.  You can change the folder and filenames with keyword arguments; the data is written to `output/date/model_name/time/` but is compiled into a single results file

write_html(
    my_sys,
    my_result,
    p1,
    p2,
    p3,
    folder = "output",
    :verbose,
    bode = [3],
)
# make the Bode plot using the third ouput (kx) only, because the Bode plot should be dimensionless, i.e., input and output should have the same units, so we plot the ratio of psring force to applied force, as a function of frequency

using EoM_X3D
animate_modes(my_sys[1], my_result[1], scale=0.2)

println("Done.")
