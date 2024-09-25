using EoM, EoM_X3D

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

#system = input_full_car_a_arm_pushrod(; u, a, b, tw, r)
#output = run_eom!(system)
# eom_draw(system)

#result = analyze(output; freq=(-1,2))
#impulse = :skip
#summarize(system, result; impulse)
#summarize(system, result; impulse, format=:html)


system = input_full_car_a_arm_pushrod(; u, a, b, tw, r, flex = true)
output = run_eom!(system)
result = analyze(output; freq=(-1,2))
impulse = :skip
summarize(system, result; impulse)
#summarize(system, result; impulse, format=:html)

# animate_modes(system, result)


(; A, B, C, D) = result.ss_eqns

vec = eigen(A).vectors
val = eigen(A).values

keep = abs.(val) .< 100π
display(keep)

V = vec[:,keep]
v = val[keep]

for i in 1:length(v)-1
    if v[i] == conj(v[i+1])
        println("mod $i and $(i+1)")
        temp1 = V[:,i] + V[:,i+1]
        temp1 ./= norm(temp1)
        temp2 = (V[:,i] - V[:,i+1]) * -1im
        temp2 ./= norm(temp2)
        V[:,i] = temp1
        V[:,i+1] = temp2 
    end
end

W = pinv(V)

AA = real(W*A*V)
BB = real(W*B)
CC = real(C*V)
DD = D

display(eigen(AA).values)





#=

using EoM_X3D
animate_modes(system, result, true)
eom_draw(system)


system = input_full_car_a_arm_pushrod(; u, a, tw, r)
sensors_animate!(system)
output = run_eom!(system)
result = analyze(output)

zofxl, zofxr = random_road(class=5, dz=0.2)
u_vec(_, t) = [zofxl(u * t), zofxl(u * t - a - b), zofxr(u * t), zofxr(u * t - a - b)]

println("Solving time history...")
t1 = 0
t2 = 20
yy = ltisim(result.ss_eqns, u_vec, (t1, t2))
t = t1:(t2-t1)/1000:t2
y = hcat(yy.(t)...)

animate_history(system, t, y)

=#

println("Done.")
