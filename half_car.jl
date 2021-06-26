using EoM

build_examples()
include(joinpath("examples", "input_ex_half_car.jl"))
temp() = input_ex_half_car(m = 2000, a = 1.5, b = 1.3, cf = 100, cr = 100, I = 2000)
my_sys, my_eqns = run_eom(temp)
my_result = analyze(my_eqns)
write_html(my_sys, my_result)

using EoM_X3D
animate_modes(my_sys[1], my_result[1])

println("Done.")
