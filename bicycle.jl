
using EoM

build_examples()
include(joinpath("examples", "input_ex_bicycle_rider.jl"))

# here you can enter your vehicle specs by name, and set the speed
temp(x) = input_ex_bicycle_rider(u = x)
vpts = 0.1:0.2:10
my_sys, my_eqns = run_eom(temp, :verbose; vpts)
my_result = analyze(my_eqns, :verbose)

write_html(my_sys, my_result)
