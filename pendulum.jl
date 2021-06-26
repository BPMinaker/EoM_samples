using EoM

build_examples()
include(joinpath("examples", "input_ex_pendulum.jl"))
my_sys, my_eqns = run_eom(input_ex_pendulum)
my_result = analyze(my_eqns)
write_html(my_sys, my_result)

using EoM_X3D
animate_modes(my_sys[1], my_result[1])

println("Done.")
