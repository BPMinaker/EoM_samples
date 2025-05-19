module quarter_car
using EoM

include(joinpath("models", "input_ex_quarter_car.jl"))

# here you can enter your vehicle specs by name
ms = 500
mu = 50
kt = 150000
cs = 500
vpts = 15000:100:30000
vpt_name = ["k" "Stiffness" "N/m"]

format = :screen
# format = :html

system = [input_ex_quarter_car(; ks=x, ms, mu, kt, cs) for x in vpts]
output = run_eom!.(system)
result = analyze.(output)

ss = :skip
impulse = :skip
summarize(system, vpts, result; ss, impulse, vpt_name, format)

end

println("Done.")
