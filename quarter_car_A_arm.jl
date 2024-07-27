using EoM

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

impulse = [0,0,0]
#summarize(system, result; impulse)
summarize(system, result; impulse, format = :html)


using EoM_X3D
animate_modes(system, result)
eom_draw(system)

system = quarter_car_a_arm_pushrod(; u, a, tw, r)
sensors_animate!(system)
output = run_eom!(system)
result = analyze(output)

zofx = random_road(class = 5)
u_vec(~, t) = zofx(u * t)

t = 0:0.005:20
y = splsim(result.ss_eqns, u_vec, t)

p = Matrix(y)'
animate_history(system, t, p)

println("Done.")