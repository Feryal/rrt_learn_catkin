import os

for n,filename in enumerate(os.listdir(".")):
	if filename[:3]=="bag":
		os.rename(filename,"bag"+str(n+39))