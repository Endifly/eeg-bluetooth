import pickle
import observeConfig

deviceName = input("deviceName (mindWave) : ")
freq = input("freq (10) : ")

# try :
fileName = '%s/%s/%s-%sHz.pkl'%(observeConfig.saveDirectory,deviceName,deviceName,freq)
f = open(fileName,'rb')
profile = pickle.load(f)
f.close()
print(profile)

plotting = input("Plot fft and rawWave? (y/n) : ")

if (plotting == "y" or plotting == "yes") :
    profile.toDiagram()
# except :
#     print("pls correct device name and freq")
