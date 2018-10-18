import numpy as np
import matplotlib.pyplot as plt

# Run this file to plot the respective figures.
# If you are creating multiple files through parallel cores, concatenate the files here before plotting.
# Example as shown in comments below.

x_1 = np.load("main_sarsa_1.npy")

#x_2 = np.load("q2_2.npy")
#x = np.concatenate((x_1,x_2), axis=0)

x = x_1
ret = np.mean(x,axis=0)

# Please change the number 200 below to the number of episodes.

#print (len(ret))

y = [(i+1) for i in range(50)]


err = np.std(x, axis=0)
#plt.plot(ret)

print (ret)
print (err)

plt.errorbar(y,ret, yerr = err,mfc = 'r',fmt='o', ecolor='g')
plt.title("Average returns over episode number")
plt.xlabel("Episode number")
plt.ylabel("Average return")
#plt.ylim([-1000,0])
plt.show()