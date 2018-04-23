from __future__ import print_function
import cv2
import glob
import os
import numpy as np
import sys
import itertools as it

def inside(r, q):
    rx, ry, rw, rh = r
    qx, qy, qw, qh = q
    return rx > qx and ry > qy and rx + rw < qx + qw and ry + rh < qy + qh


def draw_detections(img, rects, thickness = 1):
    for x, y, w, h in rects:
        # the HOG detector returns slightly larger rectangles than the real objects.
        # so we slightly shrink the rectangles to get a nicer output.
        pad_w, pad_h = int(0.15*w), int(0.05*h)
        cv2.rectangle(img, (x+pad_w, y+pad_h), (x+w-pad_w, y+h-pad_h), (0, 255, 0), thickness)

labels = []    
samples = []


#to change parameters at runtime:
#hog = cv2.HOGDescriptor(winSize,blockSize,blockStride,cellSize,nbins,derivAperture,winSigma,histogramNormType,L2HysThreshold,gammaCorrection,nlevels)
#hist = hog.compute(image,winStride,padding,locations)
#hog = cv2.HOGDescriptor()          default descriptor
#hog.save("hog.xml")         #save parameters
hog = cv2.HOGDescriptor("hog.xml")  

pospath = 'Positive/'
negpath = 'Negative/'

i = 0
# Get positive samples
for filename in glob.glob(os.path.join(pospath, '*.png')):
    #print filename
    img = cv2.imread(filename, 0)
    img = cv2.resize(img,(64, 128), interpolation = cv2.INTER_LINEAR)
    hist = hog.compute(img)
    hist.ravel()
    samples.append(hist)
    labels.append(0)
    i = i+1

print (i)

# Get negative samples
i = 0
for filename in glob.glob(os.path.join(negpath, '*.png')):
    #print filename
    img = cv2.imread(filename, 0)
    img = cv2.resize(img,(64, 128), interpolation = cv2.INTER_LINEAR)
    hist = hog.compute(img)
    hist.ravel
    samples.append(hist)
    labels.append(1)
    i = i+1

print (i)


# make numpy objects and Shuffle Samples/labels
rand = np.random.RandomState(10)
shuffle = rand.permutation(len(samples))
arrsamples = np.asarray(samples)
arrlabels = np.asarray(labels)
arrsamples = arrsamples[shuffle]
arrlabels = arrlabels[shuffle]
arrsamples.ravel
arrlabels.ravel

#train 80 test 20
idx = int(len(samples) * .8)
print (arrsamples.shape)
print(idx)
trainingsamples, testsamples = arrsamples[:idx,:], arrsamples[idx:,:]
traininglabels, testlabels = arrlabels[:idx], arrlabels[idx:]



# Create SVM classifier
svm = cv2.ml.SVM_create()
svm.setType(cv2.ml.SVM_C_SVC)
svm.setKernel(cv2.ml.SVM_LINEAR) # cv2.ml.SVM_LINEAR cv2.ml.SVM_RBF
svm.setDegree(1.0)
svm.setGamma(5.383)
#svm.setCoef0(0.0)
#svm.setC(2.67)
#svm.setNu(0.0)
#svm.setP(0.0)
#svm.setClassWeights(None)

# test 
print("training and testing")
svm.train(trainingsamples, cv2.ml.ROW_SAMPLE, traininglabels)
results = svm.predict(testsamples)
resultarr = results[1]
incorrect = 0
for i in range (0, len(resultarr)):
    #print(testlabels[i])
    if(resultarr[i] != testlabels[i]):
        incorrect = incorrect + 1

print("number of wrong guesses is ")
print(incorrect)
print("successrate is ")
print(float(len(resultarr) - incorrect) / len(resultarr))


#for seperate test set directory
'''testimages = []
testpath =  'TestSet/'
for filename in glob.glob(os.path.join(testpath, '*.png')):
    #print (filename)
    img = cv2.imread(filename, 0)
    img = cv2.resize(img,(64, 128), interpolation = cv2.INTER_LINEAR)
    hist = hog.compute(img)
    hist.ravel
    testimages.append(hist)

testarray = np.asarray(testimages)
results = svm.predict(testarray)
'''


#save the svm to data file
svm.save('svm_data.dat')
svmvec = svm.getSupportVectors()
rho = -svm.getDecisionFunction(0)[0]
svmvec = np.append(svmvec, rho)
print (svmvec)
b = svmvec[None]
b = b.T
c = np.array(b, dtype=np.float32 ) 
defaultdetector = cv2.HOGDescriptor_getDefaultPeopleDetector()
for x in np.nditer(c, op_flags=['readwrite']):
    if x == 0:
        x[...] = 1
#to compare support vectors to default, save to file
np.savetxt('customSV.out', c, delimiter=',')   # X is an array
np.savetxt('defaultSV.out', defaultdetector, delimiter=',')   # X is an array
hog.setSVMDetector(svmvec)


#test multiscale on an image in the ML directory
testimg = cv2.imread('thermalstitch.png', 0)
found, w = hog.detectMultiScale(testimg, winStride=(16,16), padding=(8,8), scale=1.05, finalThreshold=1)
print (w)
found_filtered = []
for ri, r in enumerate(found):
    for qi, q in enumerate(found):
        if ri != qi and inside(r, q):
            break
    else:
        found_filtered.append(r)

#draw_detections(testimg, found)
draw_detections(testimg, found_filtered, 3)
print('%d (%d) found' % (len(found_filtered), len(found)))
cv2.imshow('img', testimg)
ch = cv2.waitKey()
if ch == 27:
    cv2.destroyAllWindows()
