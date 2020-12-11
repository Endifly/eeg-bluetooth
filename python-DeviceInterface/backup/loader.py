from numpy import load
import matplotlib.pyplot as plt

def binary_parser(data) :
    rawValue = data[0:1024]
    freq_positive_l = data[1024:1024+513]
    fft_amp_l = data[1537:1537+513]
    fft_pw_l = data[2050:]
    return rawValue, freq_positive_l, fft_amp_l, fft_pw_l

data = load('data.npy')

[rawValue,freq_positive_l,fft_amp_l,fft_pw_l] = binary_parser(data)

fig, axs = plt.subplots(3)
fig.suptitle('fft_amp_l and fft_pw_l')
axs[0].plot(rawValue)
axs[1].plot(freq_positive_l,fft_amp_l,'go-')
axs[2].plot(freq_positive_l,fft_pw_l,'go-')
plt.show()
