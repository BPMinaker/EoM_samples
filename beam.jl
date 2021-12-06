using EoM

build_examples()
include(joinpath("examples", "input_ex_beam.jl"))

EI1 = 1200
EI2 = 3 * EI1
mpul = 10
l = 0.2
n = 2

system = input_ex_beam(; EI1, EI2, mpul, l, n)

verbose = true
output = run_eom!(system, verbose)
result = analyze(output, verbose)

summarize(system, result, verbose)

#using EoM_X3D
#animate_modes(system, result(), num = 8)

