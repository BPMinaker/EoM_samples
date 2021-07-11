using EoM

build_examples()
include(joinpath("examples", "input_ex_top.jl"))

temp(x) = input_ex_top(r=x)
vpts = 0.:0.1:10
my_sys, my_eqns=run_eom(temp, :verbose; vpts)
my_result = analyze(my_eqns, :verbose)

write_html(my_sys, my_result, :verbose; ss = [], vpt_name=["r" "Angular speed" "rad/s"])

println("Done.")
