import numpy as np
import sys

def main():
	file=sys.argv[1]
	f=open(file,"r")
	data=[]
	for line in f:
		d=eval(line)
		data.append(d)
	#print(data)
	np_data=np.array(data)
	print("mean:",np.mean(np_data,axis=0))
	print("std:",np.std(np_data,axis=0))


if __name__ == "__main__":
	main()

