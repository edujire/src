import os
with open("positions_file.dat","r") as pf:
	start_angles=[]
	start_positions=[]
	end_positions=[]
	num_test = int(pf.readline())
	for i in range(num_test):
		line = pf.readline().split()
		start_angles.append(float(line[0]))
		start_positions.append(float(line[1]))
		start_positions.append(float(line[2]))
		end_positions.append(float(line[3]))
		end_positions.append(float(line[4]))
	print(start_angles)
	print(start_positions)
	print(end_positions)
	
