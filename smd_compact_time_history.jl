using EoM, Plots
plotly()

build_examples()
include(joinpath("examples", "input_ex_smd.jl"))

k = 5
m = 1
c = 0.2
system = input_ex_smd(; k, m, c)
output = run_eom!(system)
result = analyze(output)

# equations of motion are found, now do time history solution

t = 0:0.02:20
w = 0.95 * result.omega_n[1][1]

u(x, t) = sin(2pi * w * t)
y = splsim(result.ss_eqns[1], u, t)

# time history done, now make plots

res = hcat(y...)'
u_t = u.(0, t)

lw = 2
xlabel = "Time [s]"
ylabel = "z [m], z dot [m/s], f [N]"
label = ["z" "zdot" "f"]

my_plot = plot(t, [res[:,[1, 2] ] u_t]; lw, xlabel, ylabel, label)
display(my_plot)

summarize(system, result; bode = [3])

println("Done.")
