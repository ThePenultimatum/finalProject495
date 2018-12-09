#!/usr/bin/env python2

import numpy as np
import cv2
from matplotlib import pyplot as plt

img = cv2.imread('jarvis.jpg')
'''
edges = cv.Canny(img,100,200)
plt.subplot(121),plt.imshow(img,cmap = 'gray')
plt.title('Original Image'), plt.xticks([]), plt.yticks([])
plt.subplot(122),plt.imshow(edges,cmap = 'gray')
plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
plt.show()
'''
'''
i = 331
limit = 400
while i < 340:
    edges = cv.Canny(img, 20, limit)
    plt.subplot(i),plt.imshow(edges,cmap = 'gray')
    i = i + 1
    limit = limit + 50
'''

# Resize
h, w = img.shape[:2]
frame_size = 100
if h > w:
    h_new = frame_size
    w_new = (frame_size * w) / h
else:
    w_new = frame_size
    h_new = (frame_size * h) / w    
resizes = cv2.resize(img, (h_new, w_new), cv2.INTER_AREA)

# Canny edge
edges = cv2.Canny(resizes, 200, 400)
'''
# Dilate
kernel = np.ones((3, 3), np.uint8)
dilates = cv2.dilate(edges, kernel, iterations = 2)

# Erode
erodes = cv2.erode(dilates, kernel, iterations = 2)
'''


# Convert type
py_img = img[:, :, :: -1]
py_resizes = resizes[:, :, :: -1]

# Show result
'''
plt.subplot(131), plt.imshow(py_img)
plt.title('Original Image'), plt.xticks([]), plt.yticks([])
plt.subplot(132), plt.imshow(py_resizes)
plt.title('Resize Image'), plt.xticks([]), plt.yticks([])
plt.subplot(133), plt.imshow(edges, cmap = 'gray')
plt.title('Canny Image'), plt.xticks([]), plt.yticks([])
'''
'''
plt.subplot(233), plt.imshow(dilates, cmap = 'gray')
plt.title('Dilate Image'), plt.xticks([]), plt.yticks([])
plt.subplot(234), plt.imshow(erodes, cmap = 'gray')
plt.title('Erode Image'), plt.xticks([]), plt.yticks([])
'''

traj = []
r, c = edges.shape
edges = np.array(edges)
#a = np.array([[1,2,3],[4,5,6],[7,8,9]])
#a[1,1] =100
#print(a)
#print(len(np.argwhere(np.array([[1,2,3],[4,5,6],[7,8,9]])==21)))

for i in range(1, r - 1):
    for j in range(1, c - 1):
        if edges[i, j] > 0:
            traj.append([i, j])
            edges[i, j] = 0
            # Start a line here
            m = i
            n = j
            line = True
            while line:
                next_point = np.argwhere(edges[m - 1: m + 2, n - 1: n + 2] > 0)
                #print(edges[m - 1: m + 2, n - 1: n + 2])
                #print('looking for line')
                #print(next_point)
                if len(next_point) > 0:
                    #print('find something')
                    #print(next_point)
                    m = next_point[0][0] + m - 1
                    n = next_point[0][1] + n - 1
                    #print(m, n)
                    if m > 0 and m < r - 1 and n > 0 and n < c - 1:
                        traj.append([m, n])
                        edges[m, n] = 0
                    else:
                        line = False
                else:
                    line = False   
#print(traj)     
                       
# Regenerate the plot 

plt.ion()
plt.axis([0, w_new, 0, h_new])
for item in traj:
    plt.plot(item[1], h_new - item[0], 'b.')
    plt.draw()
    plt.pause(0.0001)                   
   
plt.show()





