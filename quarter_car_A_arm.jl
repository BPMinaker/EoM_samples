using EoM
EoM.plotly()

include(joinpath("models", "input_quarter_car_a_arm_pushrod.jl"))
include(joinpath("models", "susp.jl"))
include(joinpath("models", "tire.jl"))

# here you can enter your vehicle specs by name
a = 2.65 * 0.58
tw = 1.71
r = 0.346
u = 10

system = quarter_car_a_arm_pushrod(; u, a, tw, r)

output = run_eom!(system)
result = analyze(output)

summarize(system, result)
summarize(system, result, format = :html)

using EoM_X3D
animate_modes(system, result)

println("Done.")