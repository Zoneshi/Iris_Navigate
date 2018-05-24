
pos = [1,2,3]
last_pos = [4,5,6]
vel = [0,0,0]
for i in range(0,3):
    vel[i] = pos[i]-last_pos[i]

print vel
