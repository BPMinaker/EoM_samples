using Plots
plotly()


#include("models/pac2002.jl")
include("models/magic.jl")
include("models/magic_3.jl")

gammarange = -0.1:0.01:0.1
sliprange = -15*pi/180:0.1*pi/180:15*pi/180
Zrange = 0.1:100:8000.1

#F = tire([1000,2000,3000,4000], [0,0,0,0], 0)
#display(F)

#F = tire([1000,2000,3000,4000], [0,0,0,0], 0.05)
#display(F)

#F = [tire(2000, slip, gamma) for slip in sliprange, gamma in gammarange]

#display(plot(gammarange,sliprange, F; size=(800, 600), st=:surface))


# F = [tire(Z, slip) for Z in Zrange, slip in sliprange]

# display(plot(sliprange, -Zrange, F; size=(800, 600), st=:surface))


G = [tire(Z, slip, 0) for Z in Zrange, slip in sliprange]

F = [G[i, j][1] for i in 1:length(Zrange), j in 1:length(sliprange)]
M = [G[i, j][2] for i in 1:length(Zrange), j in 1:length(sliprange)]

display(plot(sliprange, -Zrange, F; size=(800, 600), st=:surface))

display(plot(sliprange, -Zrange, M; size=(800, 600), st=:surface))



G = [tire(4500, slip, 0) for slip in sliprange]
F = [i[1] for i in G]
M = [i[2] for i in G]

p = plot(sliprange, F; size=(800, 600), label="γ=0")
q = plot(sliprange, M; size=(800, 600), label="γ=0")


G = [tire(4500, slip, 0.05) for slip in sliprange]
F = [i[1] for i in G]
M = [i[2] for i in G]

p = plot!(p, sliprange, F; size=(800, 600), label="γ=0.05")
q = plot!(q, sliprange, M; size=(800, 600), label="γ=0.05")


G = [tire(4500, slip, -0.05) for slip in sliprange]
F = [i[1] for i in G]
M = [i[2] for i in G]

p = plot!(p, sliprange, F; size=(800, 600), label="γ=-0.05")
q = plot!(q, sliprange, M; size=(800, 600), label="γ=-0.05")


display(p)
display(q)





#=

display(tire(4000, 0)) # test tire function
display(tire(4000, 0, 0)) # test tire function
display(tire(4000, 0.01, 0)) # test tire function
display(tire(4000, -0.01, 0)) # test tire function

=#

#=
F = [tire(Z, slip) for Z in Zrange, slip in sliprange]

display(plot!(sliprange, Zrange, F; size=(800, 600), st=:surface))

=#