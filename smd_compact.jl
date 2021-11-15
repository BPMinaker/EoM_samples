using EoM

build_examples()
include(joinpath("examples", "input_ex_smd.jl"))

k = 5
m = 1
c = 0.2
system = input_ex_smd(; k, m, c)
output = run_eom!(system)
result = analyze(output)

summarize(system, result; bode = [3])

println("Done.")