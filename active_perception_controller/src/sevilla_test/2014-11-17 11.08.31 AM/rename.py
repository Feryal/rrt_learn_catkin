import os

for n,filename in enumerate(os.listdir(".")):
	if filename[:3]=="bag":
		os.rename(filename,"bag"+str(n+39))

good_traj = [0,1,2,4,6,7,9,10,11,16,17,20,22,23,24,25,26,27,28,30,31,34,36,37,38,39,43]