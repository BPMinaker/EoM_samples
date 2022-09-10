using EoM

build_examples()
include(joinpath("examples", "input_ex_full_car.jl"))

m = 2000
a = 2.65 * 0.58
b = 2.65 * 0.42
tf = 1.71
tr = 1.71
cf = 100
cr = 100
Iy = 2000
system = input_ex_full_car(; m, a, b, tf, tr, cf, cr, Iy)
output = run_eom!(system)
result = analyze(output)

# summarize(system, result)
summarize(system, result, format = :html)

# using EoM_X3D
# animate_modes(system, result)

println("Done.")
