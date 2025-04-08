module smd

using EoM

include(joinpath("models", "input_ex_smd.jl"))

k = 50
m = 1
c = 0.2
system = input_ex_smd(; k, m, c)
output = run_eom!(system)
result = analyze(output)

# equations of motion are found, now do time history solution
# use a sinewave input near the natural frequency of the system to generate a resonance
ω = 0.95 * result.omega_n[1] * 2π
foft(t) = sin(ω * t)
u_vec(_, t) = [foft(t)]

t1 = 0
t2 = 10
yoft = ltisim(result.ss_eqns, u_vec, (t1, t2))

# pull the plot labels from the system information
label, ylabel = ltilabels(system)
my_plot = ltiplot(yoft; ylabel, label)
# we can display the plot like:
# display(my_plot)
# or we can add it to a vector of plots, and send it to the `summarize()` function
plots = [my_plot]
ss = :skip
summarize(system, result; ss, plots)

# we could also write to html output instead of the screen
# summarize(system, result; plots, format = :html)

end

println("Done.")
