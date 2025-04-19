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
ω = 0.95 * result.omega_n[1] * 2π
# define an input vector, even if it is only length one
u_vec(_, t) = [sin(ω * t)]
yoft = ltisim(result.ss_eqns, u_vec, (0, 10))

# pull the plot labels from the system information
label, ylabel = ltilabels(system)
plots = [ltiplot(yoft; ylabel, label)]
ss = :skip

summarize(system, result; ss, plots)

end

println("Done.")
