module roll_centre

using EoM, ForwardDiff

include(joinpath("models", "input_ex_roll_centre.jl"))

# see the input file for the list of parameters and their default values
# set the list of parameters that are not using defaults
# here the value of m and u are set, but we can add many more if we like
m = 1500
u = 80 / 3.6
cfy = 0
cry = 0
hf = 0.3
hr = 0.3

format = :screen
# format = :html

# build system description with no cornering stiffnesses
system = input_full_car_rc(; m, u, cfy, cry, hf, hr) # make sure to include all parameters here, and again below!!!
output = run_eom!(system, true)
result = analyze(output, true)

# define a nonlinear tire with load sensitivity
function tire(Z, slip)
    mtm = [1.6929, -55.2084E-6, 1.27128, 1601.8 * 180 / pi, 6494.6, 4.7966E-3 * 180 / pi, -0.3875E-3, 1.0]
    C = mtm[1]
    D = (mtm[2] * Z .+ mtm[3]) .* Z
    B = mtm[4] * sin.(2 * atan.(Z / mtm[5])) / C ./ D
    Bslip = B .* slip
    E = mtm[7] * Z .+ mtm[8]

    -D .* sin.(C * atan.((1.0 .- E) .* Bslip + E .* atan.(Bslip)))
end

# a smooth step function, 3 degrees, with its midpoint (i.e. 1.5) at t = 2
# the 0.5 terms change the range from -1...1 to 0...1
# the 4 determines how quickly the step occurs
steer(t) = 3 * (0.5 * tanh(4 * (t - 1.5)) + 0.5)

# get static tire normal loads (kN)
# assume LF, LR, RF, RR sequence
# display(getfield.(system.flex_points,:name))
Z0 = vcat(getfield.(system.flex_points[[1, 2, 5, 6]], :preload)...)

# compute applied tire force
function u_vec(x, t)
    # get sensor outputs (D=0)
    y = result.ss_eqns.C * x
    # get total normal load
    Z = Z0 - y[[1, 2, 5, 6]]
    # get slip angles from sensors, subtract steer on front
    slip = y[[3, 4, 7, 8]] .- steer(t) * [1, 0, 1, 0] * π / 180
    # compute tire force
    tire(Z, slip)
end

println("Solving time history...")
t1 = 0
t2 = 8

yoft = ltisim(result, u_vec, (t1, t2))
delta = steer.(yoft.t)

plots = []

# yaw rate
yidx = [9]
uidx = [0]
label, ylabel = ltilabels(system; yidx, uidx)
label = [label "Steer δ"]
ylabel *= ", δ [°]"
push!(plots, ltiplot(yoft, delta; ylabel, label, yidx, uidx))

# pitch angle
yidx = [12]
uidx = [0]
label, ylabel = ltilabels(system; yidx, uidx)
label = [label "Steer δ"]
ylabel *= ", δ [°]"
push!(plots, ltiplot(yoft, delta; ylabel, label, yidx, uidx))

# roll angle, slip angle, understeer angle
yidx = [11, 13]
uidx = [0]
label, ylabel = ltilabels(system; yidx, uidx)
label = [label "Understeer α_u" "Steer δ"]
ylabel *= ", α_u [°], δ [°]"
push!(plots, ltiplot(yoft, [yoft[14, :] .+ delta delta]; ylabel, label, yidx, uidx))

# G lift
yidx = [10]
uidx = [0]
label, ylabel = ltilabels(system; yidx, uidx)
label = [label "Steer δ"]
ylabel *= ", δ [°]"
push!(plots, ltiplot(yoft, delta; ylabel, label, yidx, uidx))

# lateral forces (nonlinear part)
yidx = [0]
uidx = [1, 2, 3, 4]
label, ylabel = ltilabels(system; yidx, uidx)
push!(plots, ltiplot(yoft; ylabel, label, yidx, uidx))

# get tire forces
ZZ = Z0' .- yoft[[1, 2, 5, 6], :]'
ΔZ = 0.5 * [ZZ[:, 3] - ZZ[:, 1] ZZ[:, 4] - ZZ[:, 2]]

slip = 180 / π * yoft[[3, 4, 7, 8], :]' - delta .* [1, 0, 1, 0]'

YY = hcat(yoft.u.(yoft.t)...)
acc = sum(YY, dims=1)[1, :] * 9.81 / sum(Z0)
N = sum(system.scratch.a * YY[[1, 3], :] - system.scratch.b * YY[[2, 4], :], dims=1)[1, :]

label = ["LF" "LR" "RF" "RR"]
yidx = [0]
uidx = [0]

ylabel = "Vertical forces ΣZ [N]"
push!(plots, ltiplot(yoft, ZZ; ylabel, label, yidx, uidx))

ylabel = "Tire slip α [°]"
push!(plots, ltiplot(yoft, slip; ylabel, label, yidx, uidx))

label = ["F" "R"]
ylabel = "Lateral weight transfer [N]"
push!(plots, ltiplot(yoft, ΔZ; ylabel, label, yidx, uidx))

ylabel = "Yaw moment N [Nm]"
push!(plots, ltiplot(yoft, N; ylabel, yidx, uidx))

label = ["ru" "Σf/m" "vdot"]
ylabel = "acc [m/s^2]"
push!(plots, ltiplot(yoft, [yoft[15, :] acc acc - yoft[15, :]]; ylabel, label, yidx, uidx))

# recompute cornering stiffnesses
# define the lateral force function at the actual vertical load
yf(x) = tire(Z0[1], x)
# differentiate by slip angle
dyf(x) = ForwardDiff.derivative(yf, x)
# evaluate a slip angle of zero
cfy = -dyf(0)
# repeat for rear
yr(x) = tire(Z0[2], x)
dyr(x) = ForwardDiff.derivative(yr, x)
cry = -dyr(0)

# rebuild the equations of motion using the updated cornering stiffnesses
system = input_full_car_rc(; m, u, cfy, cry, hf, hr)
output = run_eom!(system, true)
result = analyze(output, true)

bode = :skip
impulse = :skip
ss = :skip
summarize(system, result; plots, bode, ss, impulse, format)

# generate animations of the mode shapes
# using EoM_X3D
# animate_modes(system, result, true)

end

println("Done.")
