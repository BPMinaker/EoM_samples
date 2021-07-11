using EoM

build_examples()
include(joinpath("examples", "input_ex_disk.jl"))

temp(x) = input_ex_disk(u=x)
vpts = 0.1:0.1:4
my_sys, my_eqns=run_eom(temp, :verbose; vpts)
my_result = analyze(my_eqns, :verbose)

write_html(my_sys, my_result, :verbose)

println("Done.")
