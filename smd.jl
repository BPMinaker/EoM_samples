# you might find it helpful to turn on 'word wrapping' in VSCode; go to File, Preferences, Settings, and search for 'wrap'; change the setting to 'on'

# the first step is to load the `EoM` and other support libraries
using EoM, Plots
plotly() # choose plotting engine

# we ask EoM to build a folder of example systems like the spring mass damper; you can build your own input files for other systems, but EoM has many examples; you should look into those files to see the structure of an EoM input file

# the `true` flag is optional, to get feedback on the progress of `build_examples()`
build_examples(true)

# now, we load the function `input_ex_smd()`, which contains the definition of the spring mass damper system; we make Julia aware of the function by `including` the file that contains it; the `joinpath()` function inserts the appropriate separator, i.e., a forward slash or backslash, depending on the platform (Windows/Mac); you can `include` input files for systems you write yourself in the same way

include(joinpath("examples", "input_ex_smd.jl"))

# one of the cool features of Julia is that it allows `keyword arguments`, i.e., you can pass many arguments to a function, but you can include the name of the argument, so it doesn't matter if you mix up the order; the example input file `input_ex_smd.jl` allows you to redefine the values of the mass (m), the stiffness (k), and the damping (c) by defining the function like this:

# function input_ex_smd(; m = 1.0, c = 0.1, k = 10.0)

# notice the semicolon in the definition; any values we don't supply just use defaults defined in the file; so, for example, we could call:

# input_ex_smd(m = 1.2)

# and the default values of c and k would be used; the drawback is that sometime you get cases like this:

# m = 1.2
# input_ex_smd(m = m)

# this is confusing and weird, but we can get around it by using a semicolon in the call to the function, if we want to send a local value to the same variable name in a function

k = 5
m = 1
c = 0.2
system = input_ex_smd(; k, m, c)

# the advantage of this form is that now `m`, `c`, and `k` are available to use elsewhere in the code, if we wanted, and if we call `input_ex_smd()` after any changes to `m`, `c`, or `k`, the updated values are used

# we send the `system` to `run_eom()`; the optional argument tells `run_eom()` to print out progress to the screen

output = run_eom!(system, true)

println("Equations...")

# the equations of motion are stored as descriptor state space systems (A,B,C,D,E) in `output`

println(output)

# if we want to know more about the behaviour of this system, we can do some analysis on the equations using the `analyze()` function; again, the flag is optional, and instructs `analyze()` to display progress messages

result = analyze(output, true)

# the analysis converts the system to a standard state space form (A,B,C,D), where the E matrix is eliminated, the size of the A matrix is usually smaller after this happens, and almost certainly not the same as if we derived it by hand, even though it has the same properties

# `result` is also a structured variable that holds a number of vectors, such as `ss_eqns`; again, in this case there is only one set of `ss_eqns`, so it is a vector of length one

println(result)

println(result.ss_eqns)

# the results of the eigenvalue analysis are available; here we can use `display()` instead of `println()` as it formats vectors and matrices nicely; note that a second order system has two eigenvalues, but they are complex conjugates

display(result.e_val)

# we can also print the natural frequencies, the damping ratios, the time constants, and the wavelengths; in this case, there are two of each, but they are all duplicate pairs, as they are calculated from conjugate eigenvalues; we can do this with a special function, using any variable name that is returned by the `analyze()` function; in this case `result` is data, but `result(vpt)` is a function, that returns the result from a particular vpt, in this case, 0

println(result())

# now, the ODE solver library in Julia is great, and has a number of very sophisticated algorithms, and can solve just about any type of ODE; however, in this case, our problem is quite a simple one, a linear ODE; for a linear ODE, it is often preferable to use a simpler ODE solver; there is one in the EoM library called `splsim()` (for sparse linear simulation); it uses something called `discrete time` to convert the differential equation to a difference equation, which it can solve very quickly

# when using `splsim()`, we have to choose a set of uniform time points in advance, and provide a function that can compute the input at those time points; more small steps provide a more accurate solution, but take longer to do; splsim() will warn us if our time step is too big for the properties of the system; we define the time as a range, 0 to 20 seconds with steps of 0.02 seconds

t = 0:0.02:20

# define input forcing function; here we chose to excite the system near its natural frequency, making sure that our step 0.02 seconds is fine enough to get a good sample of the input; a step of 0.02 is 50 times per second, and the rule of thumb is we'd like 10 samples in any sinewave (bare minimum is 2), so we can comfortably sample a 5 Hz signal with this stepsize

# note that unlike Matlab, Julia makes a distinction between a vector of vectors and a matrix; we can acess a vector of vectors like so: a[2][3] to the get third entry in the second vector, where for a matrix a[2,3] gives the entry in the second row, third column; here, we pick the first entry in the first vector of the natural frequency, and choose an excitation frequency that's very close to that

w = 0.95 * result.omega_n[1][1]

# the input function we pass to `splsim()` has to be defined as a function of the state and time, i.e., it takes two arguments, but it doesn't actually have to use them both; in this case we only use a time dependent input

u(x, t) = sin(2pi * w * t)

# we pass the structured variable `ss_eqns` that holds the A, B, C, and D matrices, the input function, and the time vector to EoM's own linear ODE solver; it solves the equation x_dot = Ax + Bu for x, then uses y = Cx + Du to solve for y, the output vector, which in this case has three entries: z, z_dot, and kz; we could choose the initial state if we wanted ,but `splsim()` will just use zeroes if we don't specify anything else

y = splsim(result.ss_eqns[1], u, t)

# the return argument `y` is a vector of vectors, one output vector for each time point, i.e., y[1] = y(t=0), y[2] = y(t=0.02); to read `y`, we use the `hcat()` function for horizontal concatenation (i.e., sticking column vectors together to form a matrix), and the `splatting` operator `...` to read all the entries from `y`; you can think of it as a short form for [y[1] y[2] ... y[end]]; we stick all the columns together, because we want to plot rows of the resulting matrix, so we transpose `res` using the apostrophe operator

res = hcat(y...)'
u_t = u.(0, t)

# our result will have four columns: the displacement, the velocity, the spring force, and the applied force; we get the applied force by attaching the result of the input function, so we can plot them together; we use the dot operator again on the vector of time values; in this case the 0 in the input is ignored by the `u()` function, but the `sin()` function is evaluated for each entry in the `t` vector

# we can make a plot; here we plot `t` on the x axis, and on the y axis, the displacement, velocity, and applied force, which are all packed back into a matrix

println("Plotting...")


lw = 2
xlabel = "Time [s]"
ylabel = "z [m], z dot [m/s], f [N]"
label = ["z" "zdot" "f"]

p1 = plot(t, [res[:,[1, 2] ] u_t]; lw, xlabel, ylabel, label)
display(p1)

# the plot is stored and then sent to the screen using `display()`; this plot should show up in a tab in VS Code

# let's reproduce the plot, but with the excitation frequency well below and well above the natural frequency; in both cases, the displacement should be smaller; `u()` is defined as a function of `w` so all we have to do is update `w`, and `u()` will update as well

w = 0.5 * result.omega_n[1][1]
y = splsim(result.ss_eqns[1], u, t)
res = hcat(y...)'
u_t = u.(0, t)

p2 = plot(t, [res[:,[1, 2] ] u_t]; lw, xlabel, ylabel, label)
display(p2)


w = 2.0 * result.omega_n[1][1]
y = splsim(result.ss_eqns[1], u, t)
res = hcat(y...)'
u_t = u.(0, t)

p3 = plot(t, [res[:,[1, 2] ] u_t]; lw, xlabel, ylabel, label)
display(p3)

# to make a convenient result, we can send the system definition, the analysis results, and any extra plots to a helper function called `summarize()`
summarize(system, result, true; bode = [3])

# here, we make the Bode (i.e., frequency response) plot using the third ouput (kx) only, because the Bode plot should be dimensionless, i.e., input and output should have the same units, so we plot the ratio of spring force to applied force, as a function of frequency; the spring force is proportional to displacement, so we are really looking at displacement response, but in a nondimensional way; we should see a resonance near the natural frequency, as long as the damping ratio is below 0.707; note that at low frequencies, the spring force will nearly equal the applied force, so the Bode plot will tend toward 1.0, or 0 [dB] (remember the decibel is a logarithmic unit); at high frequency, the applied force changes direction so quickly, the mass doesn't have time to respond, so the motion becomes very small, i.e., 0.0 or -∞ [dB]


# alternatively, we can send the analysis results, and any extra plots to a helper function called `write_html()` to make a nice summary; look in the `outputs` folder for a subfolder with today's date, and in that folder, a `Spring Mass Damper.html` file; you can change the folder name and filename with keyword arguments `folder` and `filename` if you really want; the default filename is taken from the model name in the input file; the data is also written to individual files as `output/date/filename/time/plot_1.html`, etc., which won't get overwritten if you run the analysis again, but the main html output file does, so you can leave it open in your browser and just refresh if you rerun the simulation with new values

# write_html(system, result, true; plots = [p1, p2, p3], bode = [3])

println("Done.")
