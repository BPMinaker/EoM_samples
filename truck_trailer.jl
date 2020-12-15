
using EoM

build_examples()
include(joinpath("examples", "input_ex_truck_trailer.jl"))
# here you can enter your vehicle specs by name
temp(x) =
    input_ex_truck_trailer(u = x, m = 16975 / 9.81, I = 3508, d = 2.7, e = 2.9, h = 0.1)
#speed = 20
speed = 1:0.5:35
my_sys, my_eqns = run_eom(temp, vpts = speed, :verbose)
my_result = analyze(my_eqns, :verbose)

write_html(my_sys, my_result, bode = [2, 3, 4])

println("Done.")
