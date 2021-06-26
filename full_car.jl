
using EoM, Plots
plotly()

build_examples()
include(joinpath("examples", "input_ex_full_car.jl"))
temp() = input_ex_full_car(m = 2000, a = 2.65*0.58, b = 2.65*0.42, tf=1.94-0.23, tr=1.94-0.23, cf = 100, cr = 100, Iy = 2000)
my_sys, my_eqns = run_eom(temp)
my_result = analyze(my_eqns, :verbose)
write_html(my_sys, my_result)

using EoM_X3D
animate_modes(my_sys[1], my_result[1], scale=0.2)

println("Done.")
