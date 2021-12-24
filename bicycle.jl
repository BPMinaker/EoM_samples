using EoM

build_examples()
include(joinpath("examples", "input_ex_bicycle_rider.jl"))

f(x) = input_ex_bicycle_rider(u = x)
vpts = 0.2:0.2:10
system = f.(vpts)

verbose = (vpts .== 0.1)
output = run_eom!.(system, verbose)
result = analyze(output, true)

ss = []
bode = []
summarize(system, vpts, result, true; ss, bode)
# write_html(system, vpts, result, true)
