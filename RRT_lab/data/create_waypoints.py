import numpy as np

f = open('waypoints.txt', 'r')

#read each line of the file
# for each line, split the line into a list of strings delimited by comma
# convert each string to a float
# convert the list of strings to a numpy array
lines = f.readlines()
waypoints = []
for line in lines:
    line = line.split(',')
    line = [float(i) for i in line]
    waypoints.append(line)

waypoints = np.array(waypoints)
f.close()

# interpolate between waypoints by 100 points using np.linspace
# save the interpolated waypoints to a csv file

f = open('interpolated.csv', 'w')

for i in range(waypoints.shape[0]-1):
    x = np.linspace(waypoints[i,0], waypoints[i+1,0], 100)
    y = np.linspace(waypoints[i,1], waypoints[i+1,1], 100)
    for j in range(100):
        f.write(str(x[j]) + ',' + str(y[j]) + ',' + str(0.0) + ',' + str(0.0) + '\n')

f.close()
