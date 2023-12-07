using EoM, Plots
plotly()

include(joinpath("models", "input_ex_smd.jl"))

k = 5
m = 1
c = 0.2
system = input_ex_smd(; k, m, c)
output = run_eom!(system)
result = analyze(output)

# equations of motion are found, now do time history solution

t = 0:0.02:20
w = 0.95 * result.omega_n[1]

u(~, t) = sin(2π * w * t)
y = splsim(result.ss_eqns, u, t)

# time history done, now make plots

res = hcat(y...)'
u_t = u.(0, t)

xlabel = "Time [s]"
ylabel = "z [m], kz [N], f [N]"
label = ["z" "kz" "f"]
lw = 2
size = (800, 400)

my_plot = plot(t, [res u_t]; xlabel, ylabel, label, lw, size)
# we can display the plot like:
# display(my_plot)
# or we can add it to a vector of plots, and send it to the `summarize()` function
plots = [my_plot]

summarize(system, result; plots)

# we could also write to html output instead of the screen
# summarize(system, result; plots, format = :html)

println("Done.")
