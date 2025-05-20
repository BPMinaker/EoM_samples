module roll_centre
using EoM

# note that this mmodel has four actuators, one for each tire lateral force, so we can use an external calculation for the tire model, anything we like, e.g. a magic formula
# the tire model is defined in the function tire(Z, slip), where Z is the vertical load and slip is the slip angle
include(joinpath("models", "input_ex_roll_centre.jl"))

# see the input file for the list of parameters and their default values
# set the list of parameters that are not using defaults
# here the value of m and u are set, but we can add many more if we like
m = 1500
u = 80 / 3.6
a = 1.5
b = 1.5
cfy = 0
cry = 0
hf = 0.3
hr = 0.4
kf = 30000
kr = 30000
krf = 1500
krr = 500

# magic formula tire model parameters
mtm = [1.6929, -55.2084E-6, 1.27128, 1601.8 * 180 / pi, 6494.6, 4.7966E-3 * 180 / pi, -0.3875E-3, 1.0]

format = :screen
# format = :html

# build system description with no cornering stiffnesses
system = input_full_car_rc(; m, u, a, b, cfy, cry, hf, hr, kf, kr, krf, krr) # make sure to include all parameters here, and again below!!!
output = run_eom!(system, true)
result = analyze(output, true)

# define a nonlinear tire with load sensitivity
function tire(Z, slip)
    C = mtm[1]
    D = (mtm[2] * Z .+ mtm[3]) .* Z
    B = mtm[4] * sin.(2 * atan.(Z / mtm[5])) / C ./ D
    Bslip = B .* slip
    E = mtm[7] * Z .+ mtm[8]

    -D .* sin.(C * atan.((1.0 .- E) .* Bslip + E .* atan.(Bslip)))
end

# define a steer function, use a slowly increasing steer angle
steer(t) = t/10

# get static tire normal loads (kN)
# assume LF, LR, RF, RR sequence
# display(getfield.(system.flex_points,:name))
Z0 = vcat(getfield.(system.flex_points[[1, 2, 5, 6]], :preload)...)

# compute applied tire force
function u_vec(x, t)
    # get sensor outputs (D=0)
    y = result.ss_eqns.C * x
    # get total normal load
    Z = Z0 - y[[3, 4, 9, 10]]
    # get slip angles from sensors, subtract steer on front
    slip = y[[5, 6, 11, 12]] .- steer(t) * [1, 0, 1, 0] * π / 180
    # compute tire force
    tire(Z, slip)
end

println("Solving time history...")
t1 = 0
t2 = 30

yoft = ltisim(result, u_vec, (t1, t2))
delta = steer.(yoft.t)

println("Plotting results...")
# empty plot vector to push plots into
plots = []

# yaw rate
yidx = [13]
uidx = [0]
label = ["Steer angle δ"]
ylabel = ", δ [°]"
push!(plots, ltiplot(system, yoft, delta; ylabel, label, yidx, uidx))

# roll angle, pitch angle, slip angle, understeer angle
yidx = [15, 16, 17]
label = ["Understeer angle α_u" "Steer angle δ"]
ylabel = "Angles [°]"
push!(plots, ltiplot(system, yoft, [yoft[18, :] .+ delta delta]; ylabel, label, yidx, uidx))

# G lift
yidx = [14]
label = ["Steer angle δ"]
ylabel = ", δ [°]"
push!(plots, ltiplot(system, yoft, delta; ylabel, label, yidx, uidx))

# lateral forces 
yidx = [0]
uidx = [1, 2, 3, 4]
ylabel = "Lateral forces [N]"
push!(plots, ltiplot(system, yoft; ylabel, yidx, uidx))

# yaw moment
N = sum(yoft[[1, 2, 7, 8], :]; dims=1)[1, :]
yidx = [1, 2, 7, 8]
uidx = [0]
label = ["Total"]
ylabel = "Yaw moments [Nm]"
push!(plots, ltiplot(system, yoft, N; ylabel, label, yidx, uidx, formatter=:plain))

# plots not directly from inputs or outputs
uidx = [0]
yidx = [0]

# get tire lateral forces
YY = hcat(yoft.u.(yoft.t)...)

# find total front and rear tire forces
YF = (YY[1,:] + YY[3,:])
YR = (YY[2,:] + YY[4,:])

# get tire vertical forces
ZZ = Z0' .- yoft[[3, 4, 9, 10], :]'

# get tire slip angles
slip = 180 / π * yoft[[5, 6, 11, 12], :]' - delta .* [1, 0, 1, 0]'

# get tire forces using slip but with static normal loads
Y0 = hcat([tire.(Z0, i * π / 180) for i in eachrow(slip)]...)

# find total front and rear tire forces
YF0 = (Y0[1,:] + Y0[3,:])
YR0 = (Y0[2,:] + Y0[4,:])

# get the difference between the static max and actual tire forces
ΔYF = YF0 - YF
ΔYR = YR0 - YR

# find weight transfer
ΔZ = 0.5 * [ZZ[:, 3] - ZZ[:, 1] ZZ[:, 4] - ZZ[:, 2]]

label = ["Tire vertical force Z_lf" "Tire vertical force Z_lr" "Tire vertical force Z_rf" "Tire vertical force Z_rr"]
ylabel = "Vertical forces [N]"
push!(plots, ltiplot(system, yoft, ZZ; ylabel, label, yidx, uidx))

label = ["Front weight transfer" "Rear weight transfer"]
ylabel = "Lateral weight transfer [N]"
push!(plots, ltiplot(system, yoft, ΔZ; ylabel, label, yidx, uidx))

label = ["Front grip loss" "Rear grip loss"]
ylabel = "Lateral grip loss due to weight transfer [N]"
push!(plots, ltiplot(system, yoft, [ΔYF ΔYR]; ylabel, label, yidx, uidx))

label = ["Tire slip angle α_lf" "Tire slip angle α_lr" "Tire slip angle α_rf" "Tire slip angle α_rr"]
ylabel = "Slip angles [°]"
push!(plots, ltiplot(system, yoft, slip; ylabel, label, yidx, uidx))

acc = sum(YY, dims=1)[1, :] * 9.81 / sum(Z0)
label = ["ru" "Σf/m" "vdot"]
ylabel = "Lateral accel'n [m/s^2]"
push!(plots, ltiplot(system, yoft, [yoft[19, :] acc acc - yoft[19, :]]; ylabel, label, yidx, uidx))

println("Plotted results.")

# compute cornering stiffnesses from the magic tire model
cfy = mtm[4] * sin.(2 * atan.(Z0[1] / mtm[5])) 
cry = mtm[4] * sin.(2 * atan.(Z0[2] / mtm[5])) 

# rebuild the equations of motion using the updated cornering stiffnesses
# really only need to do this if we want to see the effect of the cornering stiffnesses on the yaw eigenvalues
system = input_full_car_rc(; m, u, a, b, cfy, cry, hf, hr, kf, kr, krf, krr)
output = run_eom!(system, false)
result = analyze(output, false)

bode = :skip
impulse = :skip
ss = :skip
summarize(system, result; plots, bode, ss, impulse, format)

# generate animations of the mode shapes
# using EoM_X3D
# animate_modes(system, result, true)

end

println("Done.")
