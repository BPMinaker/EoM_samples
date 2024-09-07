using EoM

include(joinpath("models", "input_ex_quarter_car.jl"))

# here you can enter your vehicle specs by name
ms = 500
mu = 50
kt = 150000
vpts = 15000:100:30000
cs = 500

f(x) = input_ex_quarter_car(; ks=x, ms, mu, kt, cs)
verbose = (vpts .== 15000)
system = f.(vpts)
output = run_eom!.(system, verbose)
result = analyze.(output, verbose)

ss = :skip
impulse = :skip
vpt_name = ["k" "Stiffness" "N/m"]
summarize(system, vpts, result; ss, impulse, vpt_name)
#summarize(system, vpts, result; ss, impulse, vpt_name, format = :html)

println("Done.")
