module yaw_plane

using EoM

include(joinpath("models", "input_ex_yaw_plane.jl"))

# here you can enter your vehicle specs by name, including m, Iz, a, b, cf, cr; make sure you add the property you want to set to the argument list of `input_ex_yaw_plane()` below after you set it; properties you don't set will use defaults defined in `input_ex_yaw_plane()`

dpr = 180 / π

# here we set the speed in `vpts`, which gets sent one at a time to the `f()` function, which finally sends them to the `input_ex_yaw_plane()` function, where they determine the value of `u`
vpts = 0.4:0.4:40

m = 1914
a = 1.473
b = 1.403
Iz = 2600
cf = 2 * 1437 * dpr
cr = 2 * 1507 * dpr

df = 2 * 34 * dpr
dr = 2 * 38 * dpr

ptf = df / cf
ptr = dr / cr

format = :screen
# format = :html

# define a dummy function that just calls our input function, but also adds the parameters we just set
f(x) = input_ex_yaw_plane(; u=x, m, a, b, Iz, cf, cr, ptf, ptr)

# generate our system
system = f.(vpts)

# generate the equations of motion, but many times, for every different value of forward speed
output = run_eom!.(system)

# do the eigenvalues, freq resp, etc, for each forward speed
result = analyze.(output; freq=(-1, 1))

# sensors are, in order, r, β, α_u, a_lat, y, θ, α_f, α_r
# write all the results; steady state plots of outputs 1 through 4, 7, 8 (5 and 6 don't reach steady state)
ss = [1, 1, 1, 1, 0, 0, 1, 1]
impulse = :skip
bode = :skip
summarize(system, vpts, result; ss, impulse, bode)
#summarize(system, vpts, result; ss, impulse, bode, format=:html)

# now, let's also do some time domain solutions; let's pick a speed of 50 mph, or 22.4 m/s, and get the equations of motion and the results for that speed
u = 22.4
n = findfirst(vpts .== u)
system = system[n]
system.name *= " $u m per s"
result = result[n]

#define the steer angle as a function of time, a sin w dwell input ala FMVSS 126
# a 0.7 Hz sinewave with origin at t=2 times zero everywhere except times one from t=2 for 3/4 of a wavelength
# plus a constant negative one for 0.5 seconds,starting right after the 3/4 wavelength
# plus a 0.7 Hz sinewave with origin at t=2.5 times zero everywhere except times one for the last 1/4 of a wavelength
# all times 2
steer(t) = 2 * (
sin(2π * 0.7 * (t - 2)) * EoM.pulse(t, 2, 2 + 0.75 / 0.7)
- EoM.pulse(t, 2 + 0.75 / 0.7, 2.5 + 0.75 / 0.7)
+ sin(2π * 0.7 * (t - 2.5)) * EoM.pulse(t, 2.5 + 0.75 / 0.7, 2.5 + 1 / 0.7))

# define input function to be steer but to also accept x and then ignore it, then put it in a vector
u_vec(_, t) = [steer(t)]

# define time interval
t1 = 0
t2 = 20

yoft = ltisim(result.ss_eqns, u_vec, (t1, t2))

# notation conflict, y is system output vector, but also lateral displacement
# sensors are, in order, r, β, α_u, a_lat, y, θ, α_f, α_r

# plot yaw rate vs time
yidx = [1]
label, ylabel = ltilabels(system; yidx)
p1 = ltiplot(yoft; ylabel, label, yidx)

# plot body slip angle vs time
yidx = [2]
label, ylabel = ltilabels(system; yidx)
p2 = ltiplot(yoft; ylabel, label, yidx)

# plot slip angles, understeer angle vs time
yidx = [7, 8, 3]
label, ylabel = ltilabels(system; yidx)
p3 = ltiplot(yoft; ylabel, label, yidx)

# plot lateral acceleration vs time
yidx = [4]
label, ylabel = ltilabels(system; yidx)
p4 = ltiplot(yoft; ylabel, label, yidx)

# plot path, noting that it is not even close to uniform scaling, x ~ 400 m, y ~ 2.5 m
xlabel = "x [m]"
ylabel = "y [m]"
label = ""
lw = 2 # thicker line weight
size = (800, 400)
p5 = EoM.plot(u * yoft.t, yoft[5,:]; xlabel, ylabel, label, lw, size)

plots = [p1, p2, p3, p4, p5]

# write all the results; steady state plots of outputs 1 through 4, 7, 8 (5 and 6 don't reach steady state)
ss = :skip
impulse = :skip
summarize(system, result; plots, ss, impulse, format)

#using EoM_X3D
#animate_modes(system, result)

# generate over a huge range of speeds to find characteristic speed
vpts = 0.4:0.4:140

using Interpolations

system = f.(vpts)
output = run_eom!.(system)
result = analyze.(output; freq=(-1, 1))

ss_resp = hcat(getproperty.(result,:ss_resp)...)
if maximum(yaw_plane.ss_resp[3,:]) > 0.5
    yy = LinearInterpolation(ss_resp[3,:],vpts)
    uchar = yy(0.5)
    println("Characteristic speed $(my_round(uchar)) m/s.")
    K = dpr * (a+b) * 9.81 / uchar^2
    println("Understeer gradient $(my_round(K)) degrees/g.")
end

end

println("Done.")
