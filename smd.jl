# you might find it helpful to turn on 'word wrapping' in VSCode; go to File, Preferences, Settings, and search for 'wrap'; change the setting to 'on'

# this isn't strictly necessary, but let's first create a unique variable space, so we can run different samples back to back without conflict; as a consequence, any variable defined in the module will need the module name attached if we want to read it in the REPL, e.g., our mass `m` will be `smd.m` in the REPL

module smd

# next, load the EoM support library, that defines the `run_eom!()`, `analyze()`, and `summarize()` functions

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

k = 10
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

# we choose an excitation frequency that's very close to the natural frequency, to excite resonance (note that we can use Greek characters in Julia; e.g., you can get an ω using \omega)

ω_n = result.omega_n[1]
if ω_n == 0
    ω_n = minimum(abs.(result.e_val))
end
ω = 0.95 * ω_n * 2π

# and define a force function that's a sinewave at that frequency

foft(t) = sin(ω * t)

# the input function we pass to our time history solver `ltisim()` has to be defined as a function of the state and time, i.e., it has to accept two arguments, but it doesn't actually have to use them both; in this case we only use a time dependent input so we record x in _ and ignore it; note the square brackets, as the input must be a vector, even if it only has length of one

u_vec(_, t) = [foft(t)]

# solve for 20 seconds

t1 = 0
t2 = 20

# we pass the structured variable `ss_eqns` that holds the A, B, C, and D matrices, the input function, and the time span to EoM's ltisim, it will formulate the problem and call Julia's ODE solver; it solves the equation x_dot = Ax + Bu for x, then uses y = Cx + Du to solve for y, the output vector, which in this case has entries z, kz, czdot, and mzddot; we could choose the initial state x0 if we wanted, but `ltisim()` will just use zeroes if we don't specify anything else

yoft = ltisim(result.ss_eqns, u_vec, (t1, t2))

# note that `ltisim()' returns yoft as a function handle, i.e., we can choose any time t in the interval and find yoft(t)

# our result is the displacement, the spring force, the damping force, and the inertial force; 

println("Plotting...")

# we can make a plot; here we plot `t` on the x axis, and on the y axis, the displacement, spring force, the damping force, the inertial force, and applied force, here there are some keyword arguments for the labels, etc.

label, ylabel = ltilabels(system)
p1 = ltiplot(yoft; ylabel, label)

# the plot is created and stored but not shown, we could send it to the screen using: display(p1); this plot would show up in a tab in VS Code or in a web browser tab
# or we can add it to a vector of plots, and send it to the `summarize()` function

# let's reproduce the plot, but with the excitation frequency well below and well above the natural frequency; in both cases, the displacement should be smaller; `foft()` is defined as a function of `ω` so all we have to do is update `ω`, and `foft()` will update as well

ω = 0.5 * ω_n * 2π
yoft = ltisim(result.ss_eqns, u_vec, (t1, t2))
p2 = ltiplot(yoft; ylabel, label)

ω = 2 * ω_n * 2π
yoft = ltisim(result.ss_eqns, u_vec, (t1, t2))
p3 = ltiplot(yoft; ylabel, label)

# now let's display all out results, along with the extra plots

plots = [p1, p2, p3]
summarize(system, result; plots)
# summarize(system, result; plots, format = :html)

# alternatively, we can send the analysis results, and any extra plots to html output; look in the `outputs` folder for a subfolder with today's date, and in that folder, a `Spring Mass Damper.html` file; that gets overwritten if you run the analysis again, so you can leave it open in your browser and just refresh if you rerun the simulation with new values

# write_output(system, result)

# using EoM_X3D
# animate_modes(system, result)

end

println("Done.")
