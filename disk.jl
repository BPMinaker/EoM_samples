using EoM

build_examples()
include(joinpath("examples", "input_ex_disk.jl"))

f(x) = input_ex_disk(u = x)
vpts = 0:4/50:4

system = f.(vpts)
output = run_eom!.(system)
result = analyze(output)

summarize(system, vpts, result)

#write_html(system, vpts, result)

println("Done.")

