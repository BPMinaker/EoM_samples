
using EoM

build_examples()
include(joinpath("examples", "input_ex_bricard.jl"))

system = input_ex_bricard(m = 2, l = 0.4)
output = run_eom!(system)
result = analyze(output)

summarize(system, result)

# write_html(system, result)

# using EoM_X3D
# animate_modes(system, result())

println("Done.")
