using EoM
EoM.plotly()

include(joinpath("models", "input_ex_quarter_car.jl"))

# here you can enter your vehicle specs by name
ms = 500
mu = 50
kt = 150000
vpts = 15000:80:25000
cs = 500

f(x) = input_ex_quarter_car(; ks = x, ms, mu, kt, cs)
verbose = (vpts .== 15000)
system = f.(vpts)
output = run_eom!.(system, verbose)
result = analyze.(output, verbose)

summarize(system, vpts, result, vpt_name = ["k" "Stiffness" "N/m"], ss = [0, 0, 0])
# summarize(system, vpts, result, vpt_name = ["k" "Stiffness" "N/m"], ss = [0, 0, 0], format = :html)

println("Done.")
