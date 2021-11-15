using EoM

build_examples()
include(joinpath("examples", "input_ex_half_car.jl"))

m = 2000
a = 1.5
b = 1.3
cf = 100
cr = 100
Iy = 2000
system = input_ex_half_car(; m, a, b, cf, cr, Iy)
output = run_eom!(system)
result = analyze(output)

summarize(system, result)

#write_html(system, result)

#using EoM_X3D
#animate_modes(system, result())

println("Done.")
