using EoM
include(joinpath("models", "input_quarter_car_planar_a_arm.jl"))
include(joinpath("models", "tire.jl"))
include("kinematics.jl")

function main()

    vpt = 2500:-50:-5000
    vpt_name = ["Y" "Lateral Force" "N"]

    # here you can enter your vehicle specs by name
    a = 2.65 * 0.58
    tw = 1.7
    wuo = tw / 2 - 0.05
    wlo = tw / 2 - 0.05
    r = 0.32
    u = 10

    ptr1(x) = input_quarter_car_planar_a_arm(; a, tw, r, u, Y=x, wuo, wlo)
    ptr2(x) = input_quarter_car_planar_a_arm(; a, tw, r, u, Y=x, wuo, wlo, wui=0.53, wli=0.53)
    ptr3(x) = input_quarter_car_planar_a_arm(; a, tw, r, u, Y=x, wuo, wlo, wui=0.53)

    vec = [
        [ptr1, "Quarter Car A-arm - long arms"],
        [ptr2, "Quarter Car A-arm - short arms"],
        [ptr3, "Quarter Car A-arm - mixed arms"],
    ]


    for i in vec

        system = i[1].(vpt)
        setfield!.(system, :name, i[2])
        output = run_eom!.(system)
        result = analyze.(output)

        rr, L, rc = system[1].scratch
        display(rr)

        summarize(system, vpt, result; vpt_name, ss=:skip)

    end
    # animate_modes(system[1], result[1], overwrite = false)

end

println("Starting...")
main()
println("Done.")
