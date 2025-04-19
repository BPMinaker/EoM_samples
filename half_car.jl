module half_car

using EoM

include(joinpath("models", "input_ex_half_car.jl"))

m = 2000
a = 1.5
b = 1.3
cf = 100
cr = 100
Iy = 2000

format = :screen
# format = :html

system = input_ex_half_car(; m, a, b, cf, cr, Iy)
output = run_eom!(system)
result = analyze(output)

impulse = :skip
summarize(system, result; impulse, format)

# using EoM_X3D
# animate_modes(system, result)

end

println("Done.")
