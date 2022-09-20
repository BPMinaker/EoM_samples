using EoM

build_examples()
include(joinpath("examples", "input_full_car_a_arm_pushrod.jl"))
include(joinpath("examples", "susp.jl"))
include(joinpath("examples", "tire.jl"))

# here you can enter your vehicle specs by name, and set the speed
a = 2.65 * 0.58
b = 2.65 * 0.42
tw = 1.94 - 0.23
r = 0.346
vpts = 10

f(x) = input_full_car_a_arm_pushrod(; u=x, a, b, tw, r)
system = f(vpts)
output = run_eom!(system)
result = analyze(output)

summarize(system, result)
# summarize(system, result, format = :html)

using EoM_X3D
animate_modes(system, result)
