import pickle
import matplotlib.pyplot as plt
import os
import observeConfig

def observeFreqRange(observeConfig) :
    observeList = []
    for i in observeConfig :
        lastFreq = i[1]
        for r in range(i[0],i[1]+1,i[2]) :
            observeList.append(r)
        if lastFreq not in observeList :
            observeList.append(lastFreq)
    return observeList
        
class SamplingInfo :
    def __init__(self,
    samplingRate = 512,
    samplingTime = 60,
    ) :
        self.samplingRate = samplingRate
        self.samplingTime = samplingTime
    def __str__(self) :
        return 'Sampling Info : \n\
    samplingRate %f samples/second\n\
    samplingTime %f second\n'\
    %(self.samplingRate,self.samplingTime)

class DDSSetting :
    def __init__(self,
    freq = 10
    ,amp = 0.001
    ,offset = 0
    ,duty = 0.5
    ,phase = 0
    ) :
        self.freq = freq
        self.offset = offset
        self.amp = amp
        self.duty = duty
        self.phase = phase

    def __str__(self) :
        return 'DDS Setting : \n\
    freq %f Hz\n\
    amp %f V\n\
    offset %f \n\
    duty %f\n\
    phase %f'\
    %(self.freq,self.amp,self.offset,self.duty,self.phase)

class EEGProfile :
    def __init__(self,
    rawValue,
    deviceName = observeConfig.deviceName,
    setting = DDSSetting(),
    samplingInfo = SamplingInfo(),
    ):
        self.rawValue = rawValue
        self.setting = setting
        self.samplingInfo = samplingInfo
        self.deviceName = deviceName
    
    def __str__(self) :
        return  \
'Raw Value : \n \
%s\n\
%s\n\
%s\n'\
    %(str(self.rawValue[0:1024]),self.setting,self.samplingInfo)

    def binary_parser(self) :
        rawValue = self.rawValue[0:1024]
        freq_positive_l = self.rawValue[1024:1024+513]
        fft_amp_l = self.rawValue[1537:1537+513]
        fft_pw_l = self.rawValue[2050:]
        return rawValue, freq_positive_l, fft_amp_l, fft_pw_l 
    
    def toDiagram(self) :
        [rawValue,freq_positive_l,fft_amp_l,fft_pw_l] = self.binary_parser()
        fig, axs = plt.subplots(4)
        infoStr = self.toString()
        props = dict(boxstyle='round', facecolor='wheat', alpha=1)
        axs[3].text(00, 1, infoStr,  fontsize=12,verticalalignment='top', bbox=props)
        axs[3].axis('off')
        fig.suptitle('%s - rawValue ,fft_amp_l , fft_pw_l - freq %s Hz'%(self.deviceName,self.setting.freq))
        axs[0].plot(rawValue)
        axs[1].plot(freq_positive_l,fft_amp_l,'go-')
        axs[2].plot(freq_positive_l,fft_pw_l,'go-')
        plt.show()
    
    def toString(self) :
         return  \
'Raw Value : \n \
%s\n\
%s\n\
%s\n'\
    %(str(self.rawValue[0:1024]),self.setting,self.samplingInfo)

    def save(self) :
        try :
            fileName = '%s/%s/%s-%dHz.pkl'%(observeConfig.saveDirectory,self.deviceName,self.deviceName,self.setting.freq)
            if not os.path.exists(os.path.dirname(fileName)):
                try:
                    os.makedirs(os.path.dirname(fileName))
                except OSError as exc: # Guard against race condition
                    if exc.errno != errno.EEXIST:
                        raise
            f = open(fileName,'wb')
            pickle.dump(self,f)
            f.close()
            print('save %s complete',fileName)
        except :
            print('something wrong while saving at %s',fileName)