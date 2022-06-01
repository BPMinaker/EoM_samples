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
kf = 18000
kr = 18000
cf = 1000
cr = 1000
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

# build equation of motion, analyze
output = run_eom!(system, true)
# display(getfield.(system.flex_points,:name))

result = analyze(output, true)

# get static tire normal loads (kN)
Z0 = vcat(getfield.(system.flex_points[[3, 4, 7, 8]],:preload)...)

# a smooth step function, 3 degrees, with its midpoint (i.e. 1.5) at t = 2
# the 0.5 terms change the range from -1...1 to 0...1
# the 4 determines how quickly the step occurs
function steer(t)
    3 * (0.5 * tanh(4 * (t - 2)) + 0.5)
end

# assume LF, LR, RF, RR sequence
function uu(x,t)
    # get weight transfer
    y = result.ss_eqns.C * x
    # get total normal load
    Z = Z0 - y[[1, 2, 5, 6]]
    # get slip angles
    slip = y[[3, 4, 7, 8]] .- steer(t) * [1, 0, 1, 0] * pi / 180
    # call tire force
    tire(Z, slip)
end


#=
function uu(x,t)
    [1000, -1000, -1000, 1000]
end
=#

# simple nonlinear tire with load sensitivity
function tire(Z, slip)
    -(0.94 .- 1.0e-5 .* Z) .* Z .* tanh.(10 .* slip)
end

println("Solving time history...")
t = 0:0.002:8

y, xu = splsim(result.ss_eqns, uu, t; flag = true)
y = hcat(y...)'

t = t[1:4:end]
y = y[1:4:end, :]
YY = xu'[1:4:end, end-3:end]

ZZ = Z0' .- y[:,[1, 2, 5, 6]]
slip = y[:, [3, 4, 7, 8]] * 180 / pi - steer.(t) .* [1, 0, 1, 0]' 
# YY = tire.(ZZ, pi / 180 * slip)
acc = sum(YY, dims=2)/(m + 2 * muf + 2 * mur)

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

summarize(system, result; plots, ss, bode, format = :html)

# generate animations of the mode shapes
using EoM_X3D
animate_modes(system, result)

# using EoM_TeX
# write_report(system, result, true; bode = copy(ss))


