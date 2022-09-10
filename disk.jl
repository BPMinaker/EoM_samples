using EoM

build_examples()
include(joinpath("examples", "input_ex_disk.jl"))

f(x) = input_ex_disk(u = x)
vpts = 0:3/50:3

system = f.(vpts)
output = run_eom!.(system)
result = analyze.(output)

summarize(system, vpts, result, format = :html)

println("Done.")


