# you might find it helpful to turn on 'word wrapping' in VSCode; go to File, Preferences, Settings, and search for 'wrap'; change the setting to 'on'

# the first step is to load the `EoM` and other support libraries
using EoM

# now, we load the function `input_ex_smd()`, which contains the definition of the spring mass damper system; we make Julia aware of the function by `including` the file that contains it; the `joinpath()` function inserts the appropriate separator, i.e., a forward slash or backslash, depending on the platform (Windows/Mac); you can `include` input files for systems you write yourself in the same way

include(joinpath("models", "input_ex_smd.jl"))

# one of the cool features of Julia is that it allows `keyword arguments`, i.e., you can pass many arguments to a function, but you can include the name of the argument, so it doesn't matter if you mix up the order; the example input file `input_ex_smd.jl` allows you to redefine the values of the mass (m), the stiffness (k), and the damping (c) by defining the function like this:
# function input_ex_smd(; m = 1.0, c = 0.1, k = 10.0) 
# notice the semicolon in the definition; any values we don't supply just use defaults defined in the file; so, for example, we could call:

# system = input_ex_smd(m = 1.2)

# and the default values of c and k would be used; the drawback is that sometime you get cases like this:

# m = 1.2
# system = input_ex_smd(m = m)

# this is confusing and weird, but we can get around it by using a semicolon in the call to the function, if we want to send a local value to the same variable name in a function

k = 50
m = 1
c = 0.2
system = input_ex_smd(; k, m, c)

# the advantage of this form is that now `m`, `c`, and `k` are available to use elsewhere in the code, if we wanted, and if we call `input_ex_smd(; k, m, c)` again after any changes to `m`, `c`, or `k`, the updated values are used

# the return variable `system` is a `structured` variable, i.e., a container variable that holds a lot of other variables inside it; these variables that are inside are called the properties, and you can find them using:

println("Properties of the system variable...")
println(propertynames(system))

# the variable `system` holds all the information in a vector called `item`; we can see that using a dot operator, like this:

println("Items in the system...")
println(system.item)

# we send the `system` to `run_eom!()`; the optional argument tells `run_eom!()` to print out progress to the screen; the exclamation ! doesn't actually do anything, but it is convention in Julia to name your functions with an exclamation at the end if they change the value of the input argument, which `run_eom!()` does; it will do some initial sorting and processing of the system definition, and then build the equations of motion

output = run_eom!(system, true)

# the equations of motion are stored as descriptor state space systems (A,B,C,D,E) in `output`

println("Equations of motion...")
println(output)

# if we want to know more about the behaviour of this system, we can do some analysis on the equations using the `analyze()` function; again, the flag is optional, and instructs `analyze()` to display progress messages

result = analyze(output, true)

# the analysis converts the system to a standard state space form (A,B,C,D), where the E matrix is eliminated, the size of the A matrix is usually smaller after this happens, and almost certainly not the same as if we derived it by hand, even though it has the same properties; it also does a lot of other things we will be interested in, like the eigenvalues and vectors, the natural frequencies, the impulse response, etc.

# `result` is also a structured variable that holds a number of vectors, such as `ss_eqns`

println(propertynames(result))
println(result.ss_eqns)

# the results of the eigenvalue analysis are available; here we can use `display()` instead of `println()` as it formats vectors and matrices nicely; note that a second order system has two eigenvalues, but they are complex conjugates

display(result.e_val)

# we can also print the natural frequencies, the damping ratios, the time constants, and the wavelengths; in this case, there are two of each, but they are all duplicate pairs, as they are calculated from conjugate eigenvalues

# to make a convenient result, we can send the system definition and the analysis results to a helper function called `summarize()` that prints all the results in nice tables or plots as necessary

# summarize(system, result)

# here, we make the Bode (i.e., frequency response) plot using the second ouput (kx) only, because the Bode plot should be dimensionless, i.e., input and output should have the same units, so we plot the ratio of spring force to applied force, as a function of frequency; the spring force is proportional to displacement, so we are really looking at displacement response, but in a nondimensional way; we should see a resonance near the natural frequency, as long as the damping ratio is below 0.707; note that at low frequencies, the spring force will nearly equal the applied force, so the Bode plot will tend toward 1.0, or 0 [dB] (remember the decibel is a logarithmic unit); at high frequency, the applied force changes direction so quickly that the mass doesn't have time to respond, so the motion becomes very small, i.e., 0.0 or -∞ [dB]

# before we actually call the summary function, let's add in some time history solutions too using `ltisim()'

println("Computing time history...")

# the input function we pass to `ltisim()` has to be defined as a function of the state and time, i.e., it takes two arguments, but it doesn't actually have to use them both; in this case we only use a time dependent input; we choose an excitation frequency that's very close to the natural frequency
ω = 0.95 * minimum(abs.(result.e_val))
foft(~, t) = sin(ω * t)

# solve for 20 seconds

t1 = 0
t2 = 10

# we pass the structured variable `ss_eqns` that holds the A, B, C, and D matrices, the input function, and the time span to EoM's ltisim, it will formulate the problem and call Julia's ODE solver; it solves the equation x_dot = Ax + Bu for x, then uses y = Cx + Du to solve for y, the output vector, which in this case has entries z and kz; we could choose the initial state x0 if we wanted, but `ltisim()` will just use zeroes if we don't specify anything else

yy = ltisim(result.ss_eqns, foft, (t1, t2))

# note that `ltisim()' returns yy as a function handle, i.e., we can choose any time t in the interval and find yy(t), so let's choose a range and evaluate; we use the dot operator on the vector of time values, i.e., the . before the ( tells Julia that we are taking a function that expects a scalar, and calling it on each element of a vector, and we're stacking the results together in a vector;

t = t1:(t2-t1)/1000:t2
y = hcat(yy.(t)...)'

# note that unlike Matlab, Julia makes a distinction between a vector of vectors and a matrix; we can acess a vector of vectors like so: a[2][3] to the get third entry in the second vector, where for a matrix a[2,3] gives the entry in the second row, third column; yy.(t) is a vector of vectors, i.e., the output vector at 1001 points over the timespan; the `hcat()' function converts vectors to a matrix, so y is a matrix

f = foft.(0, t)

# our result is the displacement, the spring force; we get the applied force by recomputing the input function, so we can plot them together; in this case the 0 in the input is ignored by the `foft()` function, but the `sin()` function is evaluated for each entry in the `t` vector

println("Plotting...")

# here there are some keyword arguments for the labels, etc.

xlabel = "Time [s]"
ylabel = "z [m], kz [N], mzddot [N], f [N]"
label = ["z" "kz" "mzddot" "f"]
lw = 2
size = (800, 400)

# we can make a plot; here we plot `t` on the x axis, and on the y axis, the displacement, spring force, and applied force, which are all packed back into a matrix

p1 = plot(t, [y f]; xlabel, ylabel, label, lw, size)

# the plot is created and stored but not shown, we could send it to the screen using: display(p1); this plot would show up in a tab in VS Code or in a web browser tab

# let's reproduce the plot, but with the excitation frequency well below and well above the natural frequency; in both cases, the displacement should be smaller; `foft()` is defined as a function of `ω` so all we have to do is update `ω`, and `foft()` will update as well

ω = 0.5 * minimum(abs.(result.e_val))
yy = ltisim(result.ss_eqns, foft, (t1, t2))
y = hcat(yy.(t)...)'
f = foft.(0, t)
p2 = plot(t, [y f]; xlabel, ylabel, label, lw, size)

ω = 2 * minimum(abs.(result.e_val))
yy = ltisim(result.ss_eqns, foft, (t1, t2))
y = hcat(yy.(t)...)'
f = foft.(0, t)
p3 = plot(t, [y f]; xlabel, ylabel, label, lw, size)

# now let's display all out results, along with the extra plots

plots = [p1, p2, p3]
summarize(system, result; plots)
# summarize(system, result; plots, format = :html)

# alternatively, we can send the analysis results, and any extra plots to html output; look in the `outputs` folder for a subfolder with today's date, and in that folder, a `Spring Mass Damper.html` file; that gets overwritten if you run the analysis again, so you can leave it open in your browser and just refresh if you rerun the simulation with new values

# write_output(system, result)

# using EoM_X3D
# animate_modes(system, result)

println("Done.")
