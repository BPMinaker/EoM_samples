using EoM

build_examples()
include(joinpath("examples", "input_ex_rotor.jl"))

temp(x) = input_ex_rotor(r=x)
vpts = 0.: 2pi/50 :2pi
my_sys, my_eqns=run_eom(temp, :verbose; vpts)
my_result = analyze(my_eqns, :verbose)

write_html(my_sys, my_result, ss=[], vpt_name=["r" "Angular speed" "rad/s"])

println("Done.")
