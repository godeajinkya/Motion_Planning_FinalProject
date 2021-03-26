#!/usr/bin/env python
import csv
import math
import rospy

rospy.sleep(60.)

# Open Map-Data
fp = open('mapdata.csv', 'r')
lines=fp.readlines()
matrix=lines[0].split(',')
for aa in range(len(matrix)):
    matrix[aa]=int(matrix[aa])
dim = math.sqrt(len(matrix))
dim=int(dim)

# Create Matrix from row data
m=[[0 for i in range(dim)] for j in range(dim)]
c=0
R=0.05
for a in range(dim):
    for b in range(dim):
        m[dim-a-1][b]=matrix[c]
        c=c+1

# Create obstacle matrix where obstacle is detected as well as undetected areas
h=0
for c in range(dim):
    for d in range(dim):
        if m[c][d]==100 or m[c][d]==-1:
            h=h+1

# Simultaneously merge vertically stacked obstacles

f=0
matt=[[0 for i in range(8)] for j in range(h)]
g=1.0
for c in range(dim):
    for d in range(dim):
        if m[c][d]==100 or m[c][d]==-1:
            X=c/20.0-4.5
            Y=d/20.0-5.0
            if d!=0:
                if m[c][d-1]==100 or m[c][d-1]==-1:
                    matt[f-1][0]=1000
                    matt[f][0]=X-R/2.0
                    matt[f][1]=Y-R/2.0-(g*R)
                    matt[f][2]=X+R/2.0
                    matt[f][3]=Y-R/2.0-(g*R)
                    matt[f][4]=X-R/2.0
                    matt[f][5]=Y+R/2.0
                    matt[f][6]=X+R/2.0
                    matt[f][7]=Y+R/2.0
                    g=g+1
                else:
                    g=1
                    matt[f][0]=X-R/2.0
                    matt[f][1]=Y-R/2.0
                    matt[f][2]=X+R/2.0
                    matt[f][3]=Y-R/2.0
                    matt[f][4]=X-R/2.0
                    matt[f][5]=Y+R/2.0
                    matt[f][6]=X+R/2.0
                    matt[f][7]=Y+R/2.0
            else:
                g=1.0
                matt[f][0]=X-R/2.0
                matt[f][1]=Y-R/2.0
                matt[f][2]=X+R/2.0
                matt[f][3]=Y-R/2.0
                matt[f][4]=X-R/2.0
                matt[f][5]=Y+R/2.0
                matt[f][6]=X+R/2.0
                matt[f][7]=Y+R/2.0
            f=f+1

f=0
for e in range(len(matt)):
    if matt[e][0]!=1000:
        f=f+1

matty=[[0 for i in range(8)] for j in range(f)]

# Stack horizontally stacked obstacles
f=0
for e in range(len(matt)):
    if matt[e][0]!=1000:
        matty[f]=matt[e]
        f=f+1
g=1.0
for e in range(len(matty)):
    if e>1:
        if matty[e][1]==matty[e-1][1] and matty[e][5]==matty[e-1][5]:
            matty[e-1][0]=1000
            matty[e][0]=matty[e][0]-(g*R)
            matty[e][4]=matty[e][4]-(g*R)
            g=g+1
        else:
            g=1.0

# Final Matrix with combined obstacles
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

# Export to CSV file
with open('prm_obstacles.csv', 'w') as file:
    writer = csv.writer(file)
    writer.writerow([-5, 5, -5, 5])
    writer.writerow([0.14, 0.14])
    writer.writerow([0, 0, 0, -1, 1, 1])
    for e in range(len(mat)):
        writer.writerow(mat[e])

    writer.writerow([-5, 5, -4.5, 5, -5, -5, -4.5, -5])
    writer.writerow([-4.7, 5, 4.5, 5, -4.5, 4.5, 4.5, 4.5])
    writer.writerow([4.5, 5, 5, 5, 4.5, -5, 5, -5])
    writer.writerow([-4.5, -4.5, 4.5, -4.5, -4.5, -5, 4.5, -5])
