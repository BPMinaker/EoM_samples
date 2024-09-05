### A Pluto.jl notebook ###
# v0.19.46

using Markdown
using InteractiveUtils

# This Pluto notebook uses @bind for interactivity. When running this notebook outside of Pluto, the following 'mock version' of @bind gives bound variables a default value (instead of an error).
macro bind(def, element)
    quote
        local iv = try Base.loaded_modules[Base.PkgId(Base.UUID("6e696c72-6542-2067-7265-42206c756150"), "AbstractPlutoDingetjes")].Bonds.initial_value catch; b -> missing; end
        local el = $(esc(element))
        global $(esc(def)) = Core.applicable(Base.get, el) ? Base.get(el) : iv(el)
        el
    end
end

# ╔═╡ 4e3bf4a1-c665-46b1-9bbf-247d494ba38a
import Pkg

# ╔═╡ 315e931b-e2b0-4b32-9569-cd998fee1892
Pkg.activate(".")

# ╔═╡ f9d604c6-39c6-4328-97c7-83ac16f2baa9
using PlutoUI

# ╔═╡ b3ee9ee4-2e07-4acc-9a17-397e011e907d
# ╠═╡ show_logs = false
using EoM, Polynomials

# ╔═╡ 1a22e2a5-5a13-45ae-b487-6979b13237f2
include(joinpath("models", "input_ex_smd.jl"))

# ╔═╡ abed96dc-b6a8-41ba-a6a8-712e9f5479f7
md"""
## Analyze the spring mass damper
### Load the libraries
"""

# ╔═╡ 201c2d90-fb1e-4ecc-8e3e-b827b0ada00f
Pkg.status()

# ╔═╡ 4752c1b9-9ef6-4355-b239-c8fc52efc4a9
pwd()

# ╔═╡ 55b194ba-3622-4bc1-b4ff-538cb27900dc
Pkg.status()

# ╔═╡ 982e085e-487d-4391-82b1-51217dfee1e1
md"""
### Load our spring mass damper model 
"""

# ╔═╡ 3a1a3624-b77a-468d-99f2-a0b07d3b0a80
md"""
### Use sliders to select the mass, damping, and stiffness
"""

# ╔═╡ 78e3c3af-84c4-4fab-8586-4dee1fdd8b54
@bind k Slider(1:1000)

# ╔═╡ 1c74daed-7799-43e0-9c19-a0eb1bd817e6
k

# ╔═╡ 7b7fae28-bf85-42e9-859a-2d2a5f903236
@bind c Slider(0:100)

# ╔═╡ 9422f999-11c3-4be9-ac24-f90177f0df63
c

# ╔═╡ 39aafb52-4520-460f-97e0-5d8cbf6cf927
@bind m Slider(1:10)

# ╔═╡ 3e8cd574-d8fb-445f-beee-668754fadaec
m

# ╔═╡ 452d78ed-07d3-4d6d-b4e7-fea6d6c21ad1
md"""
### Send the properties to the system definition
"""

# ╔═╡ 3c8dc822-5359-4afa-8996-2865e7dd2983
system = input_ex_smd(; k, m, c); nothing

# ╔═╡ c115fc3d-ff81-4214-983e-6ffd5e0478e4
md"""
### Generate the equations of motion
"""

# ╔═╡ 323b64ea-b645-4079-8d09-e1b18cf1a9bd
# ╠═╡ show_logs = false
output = run_eom!(system); nothing

# ╔═╡ 19fc3694-ad19-4caa-8612-d73acaef67fc
md"""
### Analyze the result 
"""

# ╔═╡ ffee97b0-3cbf-4bca-ae4a-bbe3e634c43b
# ╠═╡ show_logs = false
result = analyze(output; freq = (-2, 2)); nothing

# ╔═╡ 8d083fd2-03cf-41a5-9940-8509270593ce
result.e_val

# ╔═╡ 10623a97-dc83-4742-83fa-55b549016d10
result.omega_n

# ╔═╡ 7d8efab4-714a-483e-ac41-47fc427c7aec
result.zeta

# ╔═╡ 625beee6-4b69-44fa-9102-2e5485e00060
result.tau

# ╔═╡ 69b89a5f-dd58-48bc-adf0-7f4bc31967a7
result.lambda

# ╔═╡ f52bc5a6-e247-4a55-827a-8e61ee9de7fb
md"""
### Plot the eigenvalues and the frequency response
"""

# ╔═╡ e179ea4b-96d2-40fc-afef-52946e60228f
plot(real(result.e_val), imag(result.e_val); seriestype = :scatter, aspect_ratio = :equal, xlim = (-100, 10), xlabel = "Real part [rad/s]", ylabel = "Imaginary part [rad/s]", label ="")

# ╔═╡ 0e6e017c-b309-46b4-9f2a-f97f01ee5871
plot(result.w / 2pi, hcat(result.mag...)'[:,2]; xscale = :log10, xlim=(0.01,120), ylim=(-40,40), xticks=[0.01,0.1,1,10,100], xlabel = "Frequency [Hz]", ylabel = "Gain [dB]", label = "")

# ╔═╡ 5cfb9bab-9d86-4f99-b895-776fedaa0bb9
p = Polynomial([k, c, m])

# ╔═╡ afe7a44e-4183-461f-ac6d-45211b505373
s = -100:0.05:10

# ╔═╡ ddc2c022-a8a9-4268-988b-3b7d91e97c25
p.(s)

# ╔═╡ 6dde2c3c-e59a-4ac5-8bef-cb9f3155af7b
plot(s, p.(s); xlim=(-110,20), ylim=(-3000,1500), xlabel = "s [rad/s]", ylabel = "p(s) [N/m]", label="")

# ╔═╡ fd1fc207-ff51-4872-a426-895445568917
md"""
## Now compute the forced response
"""

# ╔═╡ 2d73d997-6883-4952-a9aa-2332f7a7c36e
md"""
### Use a sine wave applied force, and choose the frequency close the the natural frequency
"""

# ╔═╡ c549d33a-3780-45a4-81a0-fec8c37a6ed3
ω = 0.95 * minimum(abs.(result.e_val))

# ╔═╡ b271a8cc-50c7-45c2-b833-2be9187d7aea
foft(~, t) = sin(ω * t)

# ╔═╡ da55e077-1369-42c5-a780-105ca87a9e51
md"""
### Choose the start and end times
"""

# ╔═╡ 4316a8b3-2830-4231-912d-ce5c5e5a2de1
begin
t1 = 0
t2=10
end

# ╔═╡ 89ca8332-df5f-4e50-83b8-57d0c171a5ed
md"""
### Compute the forced time history solution 
"""

# ╔═╡ 520bc495-8596-4516-82a1-2ca35ec68da3
# ╠═╡ show_logs = false
yy = ltisim(result.ss_eqns, foft, (t1, t2)); nothing

# ╔═╡ 6f167dc2-967d-493a-93e3-f47976663796
md"""
### Evaluate the solution at 1001 points in the interval
"""

# ╔═╡ 5b43e131-2123-492e-bd38-348add5ae3ab
t = t1:(t2-t1)/1000:t2

# ╔═╡ 7babe13b-b5b8-462c-8592-a1411fff3618
y = hcat(yy.(t)...)'

# ╔═╡ be639309-74b0-4100-862a-8e6b3ad7e406
f = foft.(0, t);

# ╔═╡ 11157422-fe89-4d54-b4ac-13c10ec6e2b0
md"""
### Set some properties of the graph, and plot the time history
"""

# ╔═╡ 454fd4ad-b6b7-4fb1-967e-d02f9212818a
begin 
xlabel = "Time [s]"
ylabel = "z [m], kz [N], f [N]"
label = ["z" "kz" "f"]
lw = 2
end

# ╔═╡ 6f5b743f-b13a-4a03-b9e9-b8e149dd7b9d
plot(t, [y f]; xlabel, ylabel, label, lw)

# ╔═╡ Cell order:
# ╠═abed96dc-b6a8-41ba-a6a8-712e9f5479f7
# ╠═f9d604c6-39c6-4328-97c7-83ac16f2baa9
# ╠═4e3bf4a1-c665-46b1-9bbf-247d494ba38a
# ╠═201c2d90-fb1e-4ecc-8e3e-b827b0ada00f
# ╠═4752c1b9-9ef6-4355-b239-c8fc52efc4a9
# ╠═315e931b-e2b0-4b32-9569-cd998fee1892
# ╠═55b194ba-3622-4bc1-b4ff-538cb27900dc
# ╠═b3ee9ee4-2e07-4acc-9a17-397e011e907d
# ╠═982e085e-487d-4391-82b1-51217dfee1e1
# ╠═1a22e2a5-5a13-45ae-b487-6979b13237f2
# ╠═3a1a3624-b77a-468d-99f2-a0b07d3b0a80
# ╠═78e3c3af-84c4-4fab-8586-4dee1fdd8b54
# ╠═1c74daed-7799-43e0-9c19-a0eb1bd817e6
# ╠═7b7fae28-bf85-42e9-859a-2d2a5f903236
# ╠═9422f999-11c3-4be9-ac24-f90177f0df63
# ╠═39aafb52-4520-460f-97e0-5d8cbf6cf927
# ╠═3e8cd574-d8fb-445f-beee-668754fadaec
# ╠═452d78ed-07d3-4d6d-b4e7-fea6d6c21ad1
# ╠═3c8dc822-5359-4afa-8996-2865e7dd2983
# ╠═c115fc3d-ff81-4214-983e-6ffd5e0478e4
# ╠═323b64ea-b645-4079-8d09-e1b18cf1a9bd
# ╠═19fc3694-ad19-4caa-8612-d73acaef67fc
# ╠═ffee97b0-3cbf-4bca-ae4a-bbe3e634c43b
# ╠═8d083fd2-03cf-41a5-9940-8509270593ce
# ╠═10623a97-dc83-4742-83fa-55b549016d10
# ╠═7d8efab4-714a-483e-ac41-47fc427c7aec
# ╠═625beee6-4b69-44fa-9102-2e5485e00060
# ╠═69b89a5f-dd58-48bc-adf0-7f4bc31967a7
# ╠═f52bc5a6-e247-4a55-827a-8e61ee9de7fb
# ╠═e179ea4b-96d2-40fc-afef-52946e60228f
# ╠═0e6e017c-b309-46b4-9f2a-f97f01ee5871
# ╠═5cfb9bab-9d86-4f99-b895-776fedaa0bb9
# ╠═afe7a44e-4183-461f-ac6d-45211b505373
# ╠═ddc2c022-a8a9-4268-988b-3b7d91e97c25
# ╠═6dde2c3c-e59a-4ac5-8bef-cb9f3155af7b
# ╠═fd1fc207-ff51-4872-a426-895445568917
# ╠═2d73d997-6883-4952-a9aa-2332f7a7c36e
# ╠═c549d33a-3780-45a4-81a0-fec8c37a6ed3
# ╠═b271a8cc-50c7-45c2-b833-2be9187d7aea
# ╠═da55e077-1369-42c5-a780-105ca87a9e51
# ╠═4316a8b3-2830-4231-912d-ce5c5e5a2de1
# ╠═89ca8332-df5f-4e50-83b8-57d0c171a5ed
# ╠═520bc495-8596-4516-82a1-2ca35ec68da3
# ╠═6f167dc2-967d-493a-93e3-f47976663796
# ╠═5b43e131-2123-492e-bd38-348add5ae3ab
# ╠═7babe13b-b5b8-462c-8592-a1411fff3618
# ╠═be639309-74b0-4100-862a-8e6b3ad7e406
# ╠═11157422-fe89-4d54-b4ac-13c10ec6e2b0
# ╠═454fd4ad-b6b7-4fb1-967e-d02f9212818a
# ╠═6f5b743f-b13a-4a03-b9e9-b8e149dd7b9d
