using EoM

include(joinpath("models", "input_full_car_a_arm_pushrod.jl"))
include(joinpath("models", "susp.jl"))
include(joinpath("models", "tire.jl"))

# here you can enter your vehicle specs by name, and set the speed
a = 2.65 * 0.58
b = 2.65 * 0.42
tw = 1.94 - 0.23
r = 0.346
u = 10

system = input_full_car_a_arm_pushrod(; u, a, b, tw, r)
output = run_eom!(system)
result = analyze(output)

impulse = :skip
#summarize(system, result; impulse)
summarize(system, result; impulse, format = :html)


using EoM_X3D
animate_modes(system, result)
eom_draw(system)

system = input_full_car_a_arm_pushrod(; u, a, tw, r)
sensors_animate!(system)
output = run_eom!(system)
result = analyze(output)

zofxl, zofxr = random_road(class = 5, dz = 0.2)
u_vec(~, t) = [zofxl(u * t), zofxl(u * t - a - b), zofxr(u * t), zofxr(u * t - a - b)]

t1 = 0
t2 = 20
yy = ltisim(result.ss_eqns, u_vec, (t1, t2))
t = t1:(t2-t1)/1000:t2
y = hcat(yy.(t)...)

animate_history(system, t, y)

println("Done.")
