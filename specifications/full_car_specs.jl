# edit these points to select the front suspension geometry of your vehicle
ubj = [0, -0.15, 2 * r] # upper ball joint
lbj = [0, -0.1, r / 2] # lower ball joint
uaapf = [0.15, -0.4, 1.75 * r] # upper a-arm pivot, front
uaapr = [-0.15, -0.4, 1.75 * r] # upper a-arm pivot, rear
laapf = [0.15, -0.5, r / 2] # lower a-arm pivot, front
laapr = [-0.15, -0.5, r / 2] # lower a-arm pivot, rear
itre = [-0.2, -0.45, r - 0.033863] # inner tie rod end
otre = [-0.2, -0.12, r] # outer tie rod end
sle = [0, -0.2, r / 2] # spring lower end
sue = [0, -0.3, 2 * r]  # spring upper end

# don't edit this line (store the geometry data in a structure)
front = susp(;r, ubj, lbj, uaapf, uaapr, laapf, laapr, itre, otre, sle, sue)

#edit these points to select the rear suspension geometry of your vehicle
ubj = [0, -0.15, 2 * r] # upper ball joint
lbj = [0, -0.1, r / 2] # lower ball joint
uaapf = [0.15, -0.4, 1.75 * r] # upper a-arm pivot, front
uaapr = [-0.15, -0.4, 1.75 * r] # upper a-arm pivot, rear
laapf = [0.15, -0.5, r / 2] # lower a-arm pivot, front
laapr = [-0.15, -0.5, r / 2] # lower a-arm pivot, rear
itre = [-0.2, -0.45, r - 0.033863] # inner tie rod end
otre = [-0.2, -0.12, r] # outer tie rod end
sle = [0, -0.2, r / 2] # spring lower end
sue = [0, -0.3, 2 * r]  # spring upper end

# dont edit this line (store the geometry data in a structure)
rear = susp(;r, ubj, lbj, uaapf, uaapr, laapf, laapr, itre, otre, sle, sue)

#edit all of these properties as you see fit
# suspension
kf = 50000 # suspension stiffness, front
kr = 50000
cf = 1800 # suspension damping, front
cr = 1700
krf = 1000 # anti-roll bar stiffness, front
krr = 800
# weight distribution
fwf = 0.55 # front weight fraction (i.e., b/(a+b)), you can modify this from 0.05 to 0.95

