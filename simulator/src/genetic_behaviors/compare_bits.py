import os
import sys

def compare_files(file1,file2):
	count=0
	diff=0
	with open(file1,"r") as f1, open(file2,"r") as f2:
		for l1,l2 in zip(f1,f2):
			count+=1
			if l1!=l2:
				diff+=1
	print("Diferencias: "+str(diff)+" de: "+str(count))
	return diff

def print_matrix(matrix):
	for row in matrix:
		for item in row:
			print(item,end="\t")
		print(" ")


if __name__ == "__main__":

	N = int(sys.argv[1])
	behavior = sys.argv[2]
	files = []
	diff_matrix = []
	for i in range(N):
		files.append("data/avoid_"+behavior+"_"+str(i)+".dat")
	for f1 in files:
		row=[]
		for f2 in files:
			row.append(compare_files(f1,f2))
		diff_matrix.append(row)
	print_matrix(diff_matrix)
