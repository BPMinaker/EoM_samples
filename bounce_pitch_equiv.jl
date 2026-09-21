using EoM, EoM_X3D
using Plots
plotlyjs()

format = :screen
#format = :html

include(joinpath("models", "input_ex_bounce_pitch.jl"))

function main()

    u = 20

    m = 2000
    a = 1.5
    b = 1.3
    kf = 25000
    kr = 30000
    cf = 1500
    cr = 1600
    Iy = 2000

#    kr = a * kf / b
#    Iy = m*a*b

    system = input_ex_bounce_pitch(; m, a, b, kf, kr, cf, cr, Iy)
    output = run_eom!(system)

    ss =:skip
    bode = :skip
    impulse = :skip

    result = analyze(output; ss, bode, impulse)
    summarize(result; format, tex=true)

    # this is a manipulation of the state-space equations to show that they are equivalent to the equations of motion derived from Newton's laws
    # we assume Newtons equations of motion are in the form:
    # M xddot + L xdot + K x = F u + G udot
    #The state-space equations are in the form:
    # x' = Ax + Bu
    # y = Cx + Du
    # if we know that the first two outputs in y are z_G and θ, and the first two inputs in u are z_f and z_r, then we can partition the matrices as follows:
    # A = [A11 A12; A21 A22]
    # B = [B1; B2]
    # C = [C11 0; C21 C22]
    # D = [0; D2]
    # where A11 is 2x2, A12 is 2x2, A21 is 2x2, A22 is 2x2, B1 is 2x2, B2 is 2x2, C11 is 2x2, C21 is nx2, C22 is nxn, D2 is nx2
    # after partitioning, we can manipulate the equations to show equivalence of four terms

    A11 = result.ss_eqns.A[1:2, 1:2]
    A12 = result.ss_eqns.A[1:2, 3:4]
    A21 = result.ss_eqns.A[3:4, 1:2]
    A22 = result.ss_eqns.A[3:4, 3:4]

    AAA = A12*A22/A12

    B1 = result.ss_eqns.B[1:2, 1:2]
    B2 = result.ss_eqns.B[3:4, 1:2]

    C11 = result.ss_eqns.C[1:2, 1:2]
    C11inv = C11^-1

    T1 = C11*(A12*A21 - AAA*A11)*C11inv
    T2 = C11*(A11 + AAA)*C11inv
    T3 = C11*(A12*B2 - AAA*B1)
    T4 = C11*B1

    display(T1)
    display(T2)
    display(T3)
    display(T4)

    K = [kf+kr b*kr-a*kf; b*kr-a*kf a^2*kf+b^2*kr]
    L = [cf+cr b*cr-a*cf; b*cr-a*cf a^2*cf+b^2*cr]
    M = [m 0;0 Iy]
    F=[kf kr; -a*kf b*kr]
    G=[cf cr; -a*cf b*cr]

    display(-inv(M)*K)
    display(-inv(M)*L)
    display(inv(M)*F)
    display(inv(M)*G)

    cond1 = all(abs.(T1 - -inv(M)*K) .< 1e-5)
    cond2 = all(abs.(T2 - -inv(M)*L) .< 1e-5)
    cond3 = all(abs.(T3 - inv(M)*F) .< 1e-5)
    cond4 = all(abs.(T4 - inv(M)*G) .< 1e-5)

    if cond1 && cond2 && cond3 && cond4
        println("matches")
    end

end

println("Starting...")
main()
println("Done.")
