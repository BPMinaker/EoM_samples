module bounce_pitch

using EoM

include(joinpath("models", "input_ex_bounce_pitch.jl"))
m = 2000
a = 1.5
b = 1.3
kf = 25000
kr = 30000
cf = 600
cr = 500
Iy = 2000

#kr = a * kf / b
#Iy = m*a*b

system = input_ex_bounce_pitch(; m, a, b, kf, kr, cf, cr, Iy)
output = run_eom!(system)
result = analyze(output)

impulse = :skip
summarize(system, result; impulse, format = :html)

input_delay!(system, result, (a + b) / 10, [1, 2])
system.name *= " with input delay"
summarize(system, result; impulse, format = :html)

using EoM_X3D
animate_modes(system, result)

cf = 1800
cr = 2000

system = input_ex_bounce_pitch(; m, a, b, kf, kr, cf, cr, Iy)
output = run_eom!(system)
result = analyze(output)
system.name *= " time history"

zofx = random_road(class=5)

# but we need to convert to time index, where x=ut; assuming a forward speed of u=10 m/s gives
u_vec(_, t) = [zofx(10 * t), zofx(10 * t - (a + b))]

println("Solving time history...")
t1 = 0
t2 = 10
yoft = ltisim(result.ss_eqns, u_vec, (t1, t2))

# plot bounce
yidx = [1]
label, ylabel = ltilabels(system; yidx)
p1 = ltiplot(yoft; ylabel, label, yidx)

# plot pitch
yidx = [2]
label, ylabel = ltilabels(system; yidx)
p2 = ltiplot(yoft; ylabel, label, yidx)

# plot passenger motion
yidx = [3]
label, ylabel = ltilabels(system; yidx)
p3 = ltiplot(yoft; ylabel, label, yidx)

# plot suspension travel
yidx = [4, 5]
label, ylabel = ltilabels(system; yidx)
p4 = ltiplot(yoft; ylabel, label, yidx)

plots = [p1, p2, p3, p4]

bode = :skip
ss = :skip
summarize(system, result; plots, impulse, bode, ss, format = :html)

end

println("Done.")
