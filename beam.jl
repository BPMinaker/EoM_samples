using EoM

include(joinpath("models", "input_ex_beam.jl"))

mmpi = 0.0254
h = 2 * mmpi
b = 1 * mmpi
t = 0.060 * mmpi
E = 200e9 # Pa
ρ = 7850  # kg/m^3

I1 = (b * h^3 - (b - 2 * t) * (h - 2 * t)^3) / 12
I2 = (b^3 * h - (b - 2 * t)^3 * (h - 2 * t)) / 12

EI1 = E * I1
EI2 = E * I2
mpul = ρ * (b * h - (b - 2 * t) * (h - 2 * t))

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