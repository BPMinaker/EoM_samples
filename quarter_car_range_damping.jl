using EoM

include(joinpath("models", "input_ex_quarter_car.jl"))

# here you can enter your vehicle specs by name
ms = 500
mu = 50
kt = 150000
ks = 18000
vpts = 500:20:3000

f(x) = input_ex_quarter_car(; ks, ms, mu, kt, cs=x)
verbose = (vpts .== 500)
system = f.(vpts)
output = run_eom!.(system, verbose)
result = analyze.(output, verbose)

summarize(system, vpts, result, vpt_name=["c" "Damping" "Ns/m"], ss=[0, 0, 0])

println("Done.")
