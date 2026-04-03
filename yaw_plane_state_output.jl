using EoM, QuadGK, LinearAlgebra  #, MatrixEquations
include(joinpath("models", "input_ex_yaw_plane.jl"))

function main()


    dpr = 180 / π

    vpts = 0.4:0.4:40

    m = 1914 # mass
    a = 1.473 # front wheelbase
    b = 1.403 # rear wheelbase
    Iz = 2600 # inertia
    cf = 2 * 1437 * dpr # front axle cornering stiffness in N/rad
    cr = 2 * 1507 * dpr # rear axle cornering stiffness in N/rad

    df = 2 * 34 * dpr # front axle self-aligning moment stiffness in Nm/rad
    dr = 2 * 38 * dpr # rear axle self-aligning moment stiffness in Nm/rad

    ptf = df / cf # front pneumatic trail
    ptr = dr / cr # rear pneumatic trail

    format = :screen
    # format = :html

    system = [input_ex_yaw_plane(; u=x, m, a, b, Iz, cf, cr, ptf, ptr) for x in vpts]

    for st in system
        s = findall(typeof.(st.item) .== sensor .&& getfield.(st.item, :name) .!= "β" .&& getfield.(st.item, :name) .!= "r" .&& getfield.(st.item, :name) .!= "ψ")
        deleteat!(st.item, s)
    end

    output = run_eom!.(system)
    result = analyze.(output; bode=:skip, impulse=:skip)

    display(result[end].ss_eqns)

    (; A, B, C, D) = result[end].ss_eqns

    #=
    A = C * A * inv(C)
    B = C * B
    C = C * inv(C)
    D = D
    result[end].ss_eqns = EoM.ss_data(A, B, C, D)
    =#

    t1 = 2
    x0 = C \ [0, 0, 90]
    x1 = [0, 0, 0]
    Φ(t) = exp(A * t)

    function f(t)
        phiB = Φ(-t) * B
        phiB * phiB'
    end

    W, error = quadgk(f, 0, t1)

    display(W)
    display(error)

    uvec(_, t) = -B' * Φ(-t)' * (W \ x0)    #(x0 - Φ(-t1) * x1)

    res = ltisim(result[end], uvec, (0, t1), x0)

    plot1 = ltiplot(system[end], res)



    function f2(t)
        phiB = Φ(t1 - t) * B
        phiB * phiB'
    end


    W2, error2 = quadgk(f2, 0, t1)

    display(W2)
    display(error2)

    uvec2(_, t) = -B' * Φ(t1 - t)' * (W2 \ x0)

    res2 = ltisim(result[end], uvec2, (0, t1), x0)

    plot2 = ltiplot(system[end], res2)

    println("Rank W is ", rank([B A * B A^2 * B]))


    Rinv = 100
    Q = zeros(3, 3)
    P = zeros(3, 3) + I

    Poft = [zeros(3, 3) for i in 1:(t1*1000)+1]
    Poft[end] = P

    for i in (t1*1000):-1:1
        Pdot = -Poft[i+1] * A - A' * Poft[i+1] - Q + Poft[i+1] * B * Rinv * B' * Poft[i+1]
        Poft[i] = Poft[i+1] - Pdot * 0.001
    end

    temp = [Poft[i][1, 1] for i in eachindex(Poft)]
    temp2 = [Poft[i][2, 2] for i in eachindex(Poft)]
    temp3 = [Poft[i][3, 3] for i in eachindex(Poft)]

    plot3 = ltiplot(system[end], res, [temp temp2 temp3]; yidx=[0], uidx=[0])


    uvec3(x, t) = -B' * Rinv * Poft[Int(round(t * 1000))+1] * x

    res3 = ltisim(result[end], uvec3, (0, t1), x0)

    plot4 = ltiplot(system[end], res3)




    #    X, vals, F = arec(A, B, 10, C'*C, zeros(3))
    #    display(X)
    #    display(vals)
    #    display(F)
    #    uvec2(x,_) = - F * x


    # println("Calculating uvec2...")
    # function uvec2(x, t)
    #     W, _ = quadgk(f, t, t1)
    #     #        display(t)
    #     #        display(W)
    #     #        display(det(W))
    #     #        B' * (W \ (Φ(t-t1) * x1 - x))
    #     -B' * (W \ x)
    # end

    # res2 = ltisim(result[end], uvec2, (0, t1 - 0.2), x0)

    # plot2 = ltiplot(system[1], res2)

    plots = [plot1, plot2, plot3, plot4]

    summarize(system[end], result[end]; plots, format=:screen)
end

println("Starting...")
main()
println("Done.")




#    W, error = quadgk(t -> Φ(t0, t) * B * B' * Φ(t0, t)', t0, t1)

#=
tt = 0
XT = zeros(size(W))

for i in 1:Int(round((t1-t0)/0.00001))
    XT += (A*XT + XT*A' + B*B')*0.00001
    tt += 0.00001
end

display(XT)
display(tt)

display(XT - W)
=#






