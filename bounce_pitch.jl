using EoM

build_examples()
include(joinpath("examples", "input_ex_bounce_pitch.jl"))
m = 2000
a = 1.5
b = 1.3
kf = 25000
kr = 30000
cf = 0
cr = 0
Iy = 2000

#kr = a * kf / b
#Iy = m*a*b

system = input_ex_bounce_pitch(; m, a, b, kf, kr, cf, cr, Iy)
output = run_eom!(system)
result = analyze(output)

summarize(system, result)

input_delay!(system, result, (a + b) / 10, [1, 2])

summarize(system, result)

# using EoM_X3D
# animate_modes(system, result)

println("Done.")
