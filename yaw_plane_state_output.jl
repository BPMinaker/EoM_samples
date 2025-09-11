using EoM
using Interpolations
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
        s = findall(typeof.(st.item) .== sensor .&& getfield.(st.item, :name) .!= "β" .&& getfield.(st.item, :name) .!= "r")
        deleteat!(st.item, s)
    end

    output = run_eom!.(system)
    result = analyze.(output; bode=:skip)

    summarize(system, vpts, result; format)

    println("Done.")

end

main()
