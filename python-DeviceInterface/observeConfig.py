"""
saved file will use name = devinceName-freq.pkl
and save in [saveDirectory]/filename

Observe format is array of [start,end,jump]
[1,30,1] mean [1,2,3,4,5,...,30]
[31,100,5] mean [31,36,...]

eg
observeRange = [
  [1,30,1],
  [31,100,5],
  [101,256,10]
]
will compute to observe freq list 
[
  1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
  31, 37, 43, 49, 55, 61, 67, 73, 79, 85, 91, 97, 100,
  101, 111, 121, 131, 141, 151, 161, 171, 181, 191, 201, 211, 221, 231, 241, 251, 256
]
"""

deviceName="mindwave"
saveDirectory = "./profile_result88"

observeTime = 60 #second

freqRange = [
  [1,30,1],
  [31,100,5],
  [101,256,10]
]