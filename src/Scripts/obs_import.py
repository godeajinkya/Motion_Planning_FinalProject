#!/usr/bin/env python
import numpy as np
import csv

fp = open('src/mapdata.csv', 'r')
lines=fp.readlines()
matrix=lines[0].split(',')
for aa in range(len(matrix)):
    matrix[aa]=int(matrix[aa])
dim = np.sqrt(len(matrix))
dim=int(dim)
m=[[0 for i in range(dim)] for j in range(dim)]
c=0
R=0.1
for a in range(dim):
    for b in range(dim):
        m[dim-a-1][b]=matrix[c]
        c=c+1

h=0
for c in range(dim):
    for d in range(dim):
        if m[c][d]==100 or m[c][d]==-1:
            h=h+1

f=0
matt=[[0 for i in range(8)] for j in range(h)]
g=1
for c in range(dim):
    for d in range(dim):
        if m[c][d]==100 or m[c][d]==-1:
            X=c/10-4.8
            Y=d/10-4.8
            if d!=0:
                if m[c][d-1]==100 or m[c][d-1]==-1:
                    matt[f-1][0]=1000
                    matt[f][0]=X-R/2
                    matt[f][1]=Y-R/2-(g*R)
                    matt[f][2]=X+R/2
                    matt[f][3]=Y-R/2-(g*R)
                    matt[f][4]=X-R/2
                    matt[f][5]=Y+R/2
                    matt[f][6]=X+R/2
                    matt[f][7]=Y+R/2
                    g=g+1
                else:
                    g=1
                    matt[f][0]=X-R/2
                    matt[f][1]=Y-R/2
                    matt[f][2]=X+R/2
                    matt[f][3]=Y-R/2
                    matt[f][4]=X-R/2
                    matt[f][5]=Y+R/2
                    matt[f][6]=X+R/2
                    matt[f][7]=Y+R/2
            else:
                g=1
                matt[f][0]=X-R/2
                matt[f][1]=Y-R/2
                matt[f][2]=X+R/2
                matt[f][3]=Y-R/2
                matt[f][4]=X-R/2
                matt[f][5]=Y+R/2
                matt[f][6]=X+R/2
                matt[f][7]=Y+R/2
            f=f+1

f=0
for e in range(len(matt)):
    if matt[e][0]!=1000:
        f=f+1

matty=[[0 for i in range(8)] for j in range(f)]

f=0
for e in range(len(matt)):
    if matt[e][0]!=1000:
        matty[f]=matt[e]
        f=f+1
g=1
for e in range(len(matty)):
    if e>1:
        if matty[e][1]==matty[e-1][1] and matty[e][5]==matty[e-1][5]:
            matty[e-1][0]=1000
            matty[e][0]=matty[e][0]-(g*R)
            matty[e][4]=matty[e][4]-(g*R)
            g=g+1
        else:
            g=1

f=0
for e in range(len(matty)):
    if matty[e][0]!=1000:
        f=f+1

mat=[[0 for i in range(8)] for j in range(f)]

f=0
for e in range(len(matty)):
    if matty[e][0]!=1000:
        mat[f]=matty[e]
        f=f+1

print(len(mat))

with open('prm_obstacles.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow([-5, 5, -5, 5])
    writer.writerow([0.14, 0.14])
    writer.writerow([0, 0, 0, -1, -1, 1])
    for e in range(len(mat)):
        writer.writerow(mat[e])

    writer.writerow([-5, 5, -4.7, 5, -5, -5, -4.7, -5])
    writer.writerow([-4.7, 5, 4.7, 5, -4.7, 4.7, 4.7, 4.7])
    writer.writerow([4.7, 5, 5, 5, 4.7, -5, 5, -5])
    writer.writerow([-4.7, -4.7, 4.7, -4.7, -4.7, -5, 4.7, -5])
