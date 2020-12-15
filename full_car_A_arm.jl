
using EoM

build_examples()
include(joinpath("examples", "full_car_a_arm_pushrod.jl"))
include(joinpath("examples", "susp.jl"))
include(joinpath("examples", "tire.jl"))

# here you can enter your vehicle specs by name, and set the speed
temp(x) = full_car_a_arm_pushrod(u = x)
my_sys, my_eqns = run_eom(temp, vpts = 30, :verbose)
my_result = analyze(my_eqns, :verbose, decomp = false)

write_html(my_sys, my_result)

#using EoM_X3D
#animate_modes(my_sys[1], my_result[1])
