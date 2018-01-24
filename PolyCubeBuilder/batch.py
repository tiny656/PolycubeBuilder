import os

error = [8, 69, 132]
ref_model = "dengzi-245"
command = "PolyCubeBuilder.exe -map " + ref_model
for idx in range(133, 257):
    command += " dengzi-"+str(idx)
os.system(command)

# error_model = [569, 592, 686]
# command = "PolyCubeBuilder.exe -polycube "
# for idx in range(14 ,257):
#     c = command + "downbase-"+str(idx)
#     os.system(c)