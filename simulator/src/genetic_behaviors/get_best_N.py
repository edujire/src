import os
import sys
import bisect
import shutil
"""
Get the N best individuals and save them as <behavior>_<num_individual>.dat in desc order

Parameters
----
int 	N: 	number ob individuals 
string 	path:	folder containing population
string	behavior:	behavior ("fsm","hmm","mdp")
"""
def main():
	N = int(sys.argv[1])
	path = sys.argv[2]
	behavior = sys.argv[3]
	fitness_file=open(path+"/fitness_"+behavior+".dat","r")
	f_lines=fitness_file.readlines()
	i=0 #index of best_ind
	best_ind = []
	best_fitness = []
	for line in f_lines: #encontrar mejores individuos
		fit_actual=float(line)
		idx=bisect.bisect_left(best_fitness,fit_actual)
		best_ind.insert(idx,i)
		best_fitness.insert(idx,fit_actual)
		i+=1
		best_ind=best_ind[-N:]
		best_fitness=best_fitness[-N:]
	best_ind.reverse()
	best_fitness.reverse()
	fitness_file.close()
	count=0
	fitness_file=open("data/fitness_"+behavior+".dat","w")
	population_file=open("data/avoid_"+behavior+".dat","w")
	for ind in best_ind: #copiar archivos a nuevo directorio
		original_file = path+"/avoid_"+behavior+"_"+str(ind)+".dat"
		new_file = "data/avoid_"+behavior+"_"+str(count)+".dat"
		shutil.copy2(original_file,new_file)
		with open(new_file) as file: #agrega el individuo al archivo de poblacion
			file.readline()
			for line in file:
				population_file.write(line[:-1])
			print("Done")
		population_file.write("\n")	 
		fitness_file.write(str(best_fitness[count])+"\n")
		count += 1
	fitness_file.close()
	
	
if __name__ == "__main__":
	main()
