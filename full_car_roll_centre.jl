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
cfy = 40000
cry = 40000

# build system description
system = input_full_car_rc(; m, u, a, b, tf, tr, kf, kr, cf, cr, krf, krr, muf, mur, hf, hr, hG, cfy, cry)
output = run_eom!(system, true)
result = analyze(output, true)

# get static tire normal loads (kN)
# display(getfield.(system.flex_points,:name))
Z0 = vcat(getfield.(system.flex_points[[1, 2, 5, 6]], :preload)...)

# a smooth step function, 3 degrees, with its midpoint (i.e. 1.5) at t = 2
# the 0.5 terms change the range from -1...1 to 0...1
# the 4 determines how quickly the step occurs
function steer(t)
    3 * (0.5 * tanh(4 * (t - 1.5)) + 0.5) * π / 180
end

# assume LF, LR, RF, RR sequence
# compute applied tire force
function u_in(x, t)
    # get sensor outputs (D=0)
    y = result.ss_eqns.C * x
    # get total normal load
    Z = Z0 - y[[1, 2, 5, 6]]
    # get slip angles
    slip = y[[3, 4, 7, 8]] .- steer(t) * [1, 0, 1, 0]
    # compute tire force, cancel linear tire
    tire(Z, slip) + [cfy, cry, cfy, cry] .* y[[3, 4, 7, 8]]
end

# simple nonlinear tire with load sensitivity
function tire(Z, slip)
    -(1.2 .- 3.0e-5 .* Z) .* Z .* tanh.(8.2 .* slip)
end

println("Solving time history...")
t = 0:0.002:8

y = splsim(result.ss_eqns, u_in, t)
y = hcat(y...)'

t = t[1:4:end]
y = y[1:4:end, :]

ZZ = Z0' .- y[:, [1, 2, 5, 6]]
slip = y[:, [3, 4, 7, 8]] - steer.(t) .* [1, 0, 1, 0]'

YY = tire.(ZZ, slip)
acc = sum(YY, dims=2) / (m + 2 * muf + 2 * mur)
slip *= 180 / π

# set plot text, etc
lw = 2 # thicker plot lineweight
xlims = (0, Inf)

xlabel = "Time [s]"
label = ["LF" "LR" "RF" "RR"]
plots = []

ylabel = "Tire slip α [degree]"
push!(plots, plot(t, slip; xlabel, ylabel, label, lw, xlims))

ylabel = "Lateral forces [N]"
push!(plots, plot(t, YY; xlabel, ylabel, label, lw, xlims))

ylabel = "Yaw moment [Nm]"
push!(plots, plot(t, sum(a * YY[:, [1, 3]] - b * YY[:, [2, 4]], dims=2); xlabel, ylabel, label, lw, xlims))

ylabel = "Vertical forces [N]"
push!(plots, plot(t, ZZ; xlabel, ylabel, label, lw, xlims, ylims=(0, Inf)))

label = ["F" "R"]
ylabel = "Lateral weight transfer [N]"
push!(plots, plot(t, 0.5 * [ZZ[:, 3] - ZZ[:, 1] ZZ[:, 4] - ZZ[:, 2]]; xlabel, ylabel, label, lw, xlims))

label = "G Lift [m]"
ylabel = "G Lift [m]"
push!(plots, plot(t, y[:, 10]; xlabel, ylabel, label, lw, xlims))

label = ["Steer δ" "Roll ϕ" "Pitch θ" "Slip β" "Understeer"]
ylabel = "Angles [degree]"
push!(plots, plot(t, [180 / π * steer.(t) y[:, 11:13] -y[:, 9] * (a + b) / u + 180 / π * steer.(t)]; xlabel, ylabel, label, lw, xlims))

label = ["ru" "Σf/m" "vdot"]
ylabel = "acc [m/ss]"
push!(plots, plot(t, [y[:, 14] acc acc - y[:, 14]]; xlabel, ylabel, label, lw, xlims))


summarize(system, result; plots)
# summarize(system, result; plots, format = :html)

# generate animations of the mode shapes
# using EoM_X3D
# animate_modes(system, result, true)

println("Done.")
