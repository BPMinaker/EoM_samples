using EoM

include(joinpath("models", "input_ex_smd.jl"))

k = 50
m = 1
c = 0.2
system = input_ex_smd(; k, m, c)
output = run_eom!(system)
result = analyze(output)

# equations of motion are found, now do time history solution

ω = 0.95 * result.omega_n[1]
u(~, t) = sin(2π * ω * t)

t1 = 0
t2 = 10
yy = ltisim(result.ss_eqns, u, (t1, t2))

t = t1:(t2-t1)/1000:t2
y = hcat(yy.(t)...)'

# time history done, now make plots
res = [y u.(0, t)]

xlabel = "Time [s]"
ylabel = "z [m], kz [N], mzddot [N], f [N]"
label = ["z" "kz" "mzddot" "f"]
lw = 2
size = (800, 400)


my_plot = plot(t, res; xlabel, ylabel, label, lw, size)
# we can display the plot like:
# display(my_plot)
# or we can add it to a vector of plots, and send it to the `summarize()` function
plots = [my_plot]

ss = :skip
impulse = :skip
summarize(system, result; ss, impulse, plots)

# we could also write to html output instead of the screen
# summarize(system, result; plots, format = :html)

println("Done.")
