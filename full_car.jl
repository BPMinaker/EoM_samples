
using EoM, Plots
plotly()

build_examples()
include(joinpath("examples", "input_ex_full_car.jl"))

m = 2000
a = 2.65*0.58
b = 2.65*0.42
tf=1.94-0.23
tr=1.94-0.23
cf = 100
cr = 100
Iy = 2000

temp() = input_ex_full_car(;m, a, b, tf, tr, cf, cr, Iy)
my_sys, my_eqns = run_eom(temp)
my_result = analyze(my_eqns, :verbose)
write_html(my_sys, my_result)

using EoM_X3D
animate_modes(my_sys[1], my_result[1], scale=0.2)

println("Done.")
