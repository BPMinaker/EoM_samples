
using EoM

build_examples()
include(joinpath("examples", "input_ex_bounce_pitch.jl"))
m = 2000
a = 1.5
b = 1.3
kf = 25000
kr = 30000
cf = 0
cr = 0
I = 2000

temp() = input_ex_bounce_pitch(;m, a, b, kf, kr, cf, cr, I)
my_sys, my_eqns = run_eom(temp)
my_result = analyze(my_eqns)
write_html(my_sys, my_result)

using EoM_X3D
animate_modes(my_sys[1], my_result[1])

println("Done.")
