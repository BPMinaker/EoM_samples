
using EoM

build_examples()
include(joinpath("examples", "full_car_a_arm_pushrod.jl"))
include(joinpath("examples", "susp.jl"))
include(joinpath("examples", "tire.jl"))

# here you can enter your vehicle specs by name, and set the speed
a = 2.65*0.58
b = 2.65*0.42
tw=1.94-0.23
r=0.346

temp(x) = full_car_a_arm_pushrod(;u = x, a, b, tw, r)
my_sys, my_eqns = run_eom(temp, vpts = 1, :verbose)
my_result = analyze(my_eqns, :verbose, decomp = true)

write_html(my_sys, my_result, :verbose)

using EoM_X3D
animate_modes(my_sys[1], my_result[1], :verbose)
