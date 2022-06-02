using EoM, Plots
plotly()

build_examples()
include(joinpath("examples", "input_ex_roll_centre.jl"))

m = 1800
u = 20
a = 1.5
b = 1.5
tf = 1.6
tr = 1.6
kf = 30000
kr = 30000
cf = 2500
cr = 2500
krf = 500
krr = 500
muf = 20
mur = 20
hf = 0.2
hr = 0.2
hG = 0.5

# build system description
# note that the linear tire cornering stiffness defaults to zero
# but we have an actuator to apply the nonlinear tire force model
system = input_full_car_rc(; m, u, a, b, tf, tr, kf, kr, cf, cr, krf, krr, muf, mur, hf, hr, hG, cfy = 0, cry = 0)
output = run_eom!(system, true)
result = analyze(output, true)

# get static tire normal loads (kN)
# display(getfield.(system.flex_points,:name))
Z0 = vcat(getfield.(system.flex_points[[1, 2, 5, 6]],:preload)...)

# a smooth step function, 3 degrees, with its midpoint (i.e. 1.5) at t = 2
# the 0.5 terms change the range from -1...1 to 0...1
# the 4 determines how quickly the step occurs
function steer(t)
    3 * (0.5 * tanh(4 * (t - 1.5)) + 0.5)
end

# assume LF, LR, RF, RR sequence
# compute applied tire force
function u_in(x,t)
    # get sensor outputs (D=0)
    y = result.ss_eqns.C * x
    # get total normal load
    Z = Z0 - y[[1, 2, 5, 6]]
    # get slip angles
    slip = y[[3, 4, 7, 8]] .- steer(t) * [1, 0, 1, 0] * pi / 180
    # compute tire force
    tire(Z, slip)
end

# simple nonlinear tire with load sensitivity
function tire(Z, slip)
    -(1.2 .- 3.0e-5 .* Z) .* Z .* tanh.(15 .* slip)
end

println("Solving time history...")
t = 0:0.002:8

y, xu = splsim(result.ss_eqns, u_in, t; flag = true)
y = hcat(y...)'

t = t[1:4:end]
y = y[1:4:end, :]
YY = xu'[1:4:end, end-3:end]
acc = sum(YY, dims=2)/(m + 2 * muf + 2 * mur)
ZZ = Z0' .- y[:,[1, 2, 5, 6]]
slip = y[:, [3, 4, 7, 8]] * 180 / pi - steer.(t) .* [1, 0, 1, 0]' 

# set plot text, etc
lw = 2 # thicker plot lineweight
xlims = (0, Inf)

xlabel = "Time [s]"
label = ["LF" "LR" "RF" "RR"]
ylabel = "Vertical forces [N]"
plots = [plot(t, ZZ; xlabel, ylabel, label, lw, xlims, ylims = (0, Inf))]

ylabel = "Tire slip angles [degree]"
push!(plots, plot(t, slip; xlabel, ylabel, label, lw, xlims))

ylabel = "Lateral forces [N]"
push!(plots, plot(t, YY; xlabel, ylabel, label, lw, xlims))

label = ["F" "R"]
ylabel = "Lateral weight transfer [N]"
push!(plots, plot(t, [ZZ[:,3] - ZZ[:,1] ZZ[:,4] - ZZ[:,2]]; xlabel, ylabel, label, lw, xlims))

label = "G Lift [m]"
ylabel = "G Lift [m]"
push!(plots, plot(t, y[:, 10]; xlabel, ylabel, label, lw, xlims))

label = ["Steer" "Roll" "Pitch" "Slip" "Understeer"]
ylabel = "Angles [degree]"
push!(plots, plot(t, [steer.(t) y[:, 11:13] -y[:, 9] * (a + b) / u + steer.(t)]; xlabel, ylabel, label, lw, xlims))

label = ["ru" "Σf/m" "vdot"]
ylabel = "acc [m/ss]"
push!(plots, plot(t, [y[:, 14] acc acc - y[:, 14]]; xlabel, ylabel, label, lw, xlims))

ss = zeros(size(result.ss_eqns.D))
ss[15:16, 1:4] = ones(2, 4)
bode = ss

# note that the yaw mode shows no eigenvalue because the cornering stiffness is set to zero
# we know the slope of the tire model
# so we can add a linear tire model and recompute the eigenvalues
# note: d tanh ax / dx = asech^2 ax 
# but sech(0) = 1, so d tanh ax|x=0 = a
# use the fact that tanh(x->inf) = 1 to compute the normal force term only
cy = -15 * tire(Z0[1:2], [1e6, 1e6])
system2 = input_full_car_rc(; m, u, a, b, tf, tr, kf, kr, cf, cr, krf, krr, muf, mur, hf, hr, hG, cfy = cy[1], cry = cy[2])
output2 = run_eom!(system2, true)
result2 = analyze(output2, true)

# let's keep the initial steady state and freq resp results
result2.w = result.w
result2.freq_resp = result.freq_resp
result2.mag = result.mag
result2.phase = result.phase
result2.ss_resp = result.ss_resp

summarize(system2, result2; plots, ss, bode, format = :html)

# generate animations of the mode shapes
using EoM_X3D
animate_modes(system2, result2, true)







println("Done.")
