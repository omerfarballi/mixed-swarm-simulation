num_of_ugv=40
the_path = ['afsdasfd']*40
the_target=['1542']*40


import os
os.mkdir("İKA'lar için mesaj dosyaları")
path=os.getcwd()
path=path.replace("\\","/")

def com(num_of_ugv,the_path,the_target):
    for i in range(num_of_ugv):
        f = open(f"{path}/İKA'lar için mesaj dosyaları/ugv_{i}.txt", "w")
        f.write(the_path[i])
        f.write("\n")
        f.write(the_target[i])


com(num_of_ugv,the_path,the_target)