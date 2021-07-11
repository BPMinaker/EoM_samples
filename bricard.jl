
using EoM, EoM_X3D

build_examples()
include(joinpath("examples", "input_ex_bricard.jl"))

temp() = input_ex_bricard(m=2,l=0.4)
my_sys, my_eqns = run_eom(temp)
my_result = analyze(my_eqns)

animate_modes(my_sys[1], my_result[1])

write_html(my_sys, my_result)

println("Done.")
