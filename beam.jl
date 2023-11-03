using EoM

include(joinpath("models", "input_ex_beam.jl"))

h = 2 * 0.0254
b = 1 * 0.0254
t = 0.060 * 0.0254

EI1 = 200e9 * (b*h^3 - (b - 2*t) * (h - 2*t)^3) / 12
EI2 = 200e9 * (b^3*h - (b - 2*t)^3 * (h - 2*t)) / 12
mpul = 7850 * (b*h - (b - 2*t) * (h - 2*t))
l = 1
n = 1

system = input_ex_beam(; EI1, EI2, mpul, l, n)
output = run_eom!(system, true)
result = analyze(output, true)

bode = [1 0; 0 1]
verbose = true
summarize(system, result; bode)
# summarize(system, result; bode, format = :html)

# using EoM_X3D
# animate_modes(system, result)

println("Done.")