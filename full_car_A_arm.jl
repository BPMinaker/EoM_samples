module full_car

using EoM

include(joinpath("models", "input_full_car_a_arm_pushrod.jl"))
include(joinpath("models", "susp.jl"))
include(joinpath("models", "tire.jl"))
include(joinpath("models", "drive.jl"))

# here you can enter your vehicle specs by name, and set the speed
a = 2.65 * 0.58
b = 2.65 * 0.42
tw = 1.94 - 0.23
r = 0.346
u = 10

format = :screen
# format = :html

system = input_full_car_a_arm_pushrod(; u, a, b, tw, r)
output = run_eom!(system)
result = analyze(output, freq=(-1, 2))

impulse = :skip
summarize(system, result; impulse, format)

using EoM_X3D
# animate_modes(system, result, true)
eom_draw(system)

system = input_full_car_a_arm_pushrod(; u, a, tw, r)
sensors_animate!(system)
output = run_eom!(system)
result = analyze(output)

zofxl, zofxr = random_road(class=5, dz=0.2)
u_vec(_, t) = [0, 0, zofxl(u * t), zofxl(u * t - a - b), zofxr(u * t), zofxr(u * t - a - b)]

println("Solving time history...")
t1 = 0
t2 = 20
y = ltisim(result.ss_eqns, u_vec, (t1, t2))

animate_history(system, y.t, y)

end

println("Done.")
