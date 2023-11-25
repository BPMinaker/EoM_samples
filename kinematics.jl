
function kinematics(; r_A = [0, 0], r_B, r_C, r_D, r_E = [0.1, -0.3], theta = 0, A_arm = true)


    function orth(x)

        [-x[2], x[1]]

    end


    # A_arm = true


    if A_arm == true

        r_BA = r_B - r_A
        r_DB = r_D - r_B
        r_DC = r_D - r_C
        r_ED = r_E - r_D

        A = [orth(r_BA) orth(r_DB)]
        omega = A \ orth(r_DC)
        # println("omega")
        # display(omega)

        alpha = A \ ([r_BA r_DB] * (omega .^ 2) - r_DC)

        # alpha = A \ (  omega[1]^2 * r_BA + omega[2]^2 * r_DB - r_DC )
        # println("alpha")
        # display(alpha)

    else

        u = [sin(theta), -cos(theta)]
        r_DB = r_D - r_B
        r_DC = r_D - r_C
        r_ED = r_E - r_D

        A = [u orth(r_DB)]
        omega = A \ orth(r_DC)
        # println("omega")
        # display(omega)

        alpha = A \ (-2 * orth(omega[1] * u) * omega[2] + omega[2]^2 * r_DB - r_DC)
        # println("alpha")
        # display(alpha)

    end

    v_E = orth(r_DC) + orth(r_ED) * omega[2]
    # println("v_E")
    # display(v_E)

    a_E = -r_DC + alpha[2] * orth(r_ED) - omega[2]^2 * r_ED
    # println("a_E")
    # display(a_E)


    R = v_E' * v_E * orth(v_E) / (a_E' * orth(v_E))
    #println("Centre of curvature")
    #display(R)

    r = sqrt(R' * R)
    #println("Radius of curvature")
    #display(r)

    u_E = v_E / sqrt(v_E' * v_E)

    IC = orth(v_E) / omega[2]
    L = sqrt(IC' * IC)
    #println("Virtual swing axle length")
    #display(L)

    RC = -orth(u_E)[2] / orth(u_E)[1] * r_E[1]
    #println("Roll centre height")
    #display(RC)

    r, L, RC

end


# Lambda config 1
# a = 0.2
# r_A = [0.0, 4*a]
# r_B = [a, 4*a]
# r_C = [0.0, 2*a]
# r_D = [2.5*a, 2*a]
# r_E = [4*a, 0.0]


# Lambda config 2
# a = 0.2
# r_A = [0.0, 2*a]
# r_B = [a, 3*a]
# r_C = [0.0, 0.0]
# r_D = [2*a, 1.5*a]
# r_E = [4*a, 0.0]


# r_A = [0.6, 0.45]
# r_B = [0.7, 0.5]
# r_C = [0.4, 0.1]
# r_D = [0.7, 0.1]
# r_E = [0.8, 0.0]


# theta = 0.1
# r_B = [0.6, 0.5]
# r_C = [0.4, 0.1]
# r_D = [0.7, 0.1]
# r_E = [0.8, 0.0]


# # println("u_E")
# # display(u_E)

# a_En = orth(u_E) * orth(u_E)' * a_E     #a_E - (a_E' * u_E) * u_E
# # println("a_n")
# # display(a_En)

# a_n = sqrt(a_En' * a_En)
# # println("|a_n|")
# # display(a_n)

# R = v_E' * v_E / a_n * -sign(a_En[1])
# println("Radius of curvature")
# display(R)



