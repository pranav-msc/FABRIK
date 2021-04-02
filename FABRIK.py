##  3 link RRR planar manipulator

import numpy as np
import math
from operator import add,sub


def euclidean(lst1,lst2):
    lst=list(map(sub,lst2,lst1))
    lst=[i**2 for i in lst]
    return math.sqrt(sum(lst))


def scalarMul(scalar,list):
    lst=[scalar*i for i in list]
    return lst


inital_position=np.zeros(shape=(4,2))


print("Enter initial position of manipulator and end effector(3 co-ordinates for manipulator,1 for end effector)")
for i in range(4):
        print("Enter {} coordinate: ".format(i+1))
        inital_position[i][0],inital_position[i][1]=list(map(float, input().split()))


goal=list(map(float,input("Enter coordinates of goal: ").split()))

#link_length=int(input("Enter length of all links: "))
print("iteration begins.....")



link_lengths=[]

tol=0.01
for i in range(inital_position.shape[0]-1):
    link_lengths.append(euclidean(inital_position[i+1],inital_position[i]))

diff=euclidean(goal,inital_position[0])
itr=0
if(diff>sum(link_lengths)):
    print("not reachable")
    for i in range(inital_position.shape[0]-1):
        ##finding distance b/w target and join
        r=euclidean(goal,inital_position[i])
        lamb=link_lengths[i]/r
        inital_position[i+1]=list(map(add,((1-lamb)*inital_position[i]),scalarMul(lamb,goal)))
    print(inital_position[3])
else:
    ##if the target is reachable
    b=np.copy(inital_position[0])
    curr_tol=euclidean(goal,inital_position[3])
    while(curr_tol>tol):
        inital_position[3]=goal
        ##Traveling backward
        for i in range(inital_position.shape[0]-2,-1,-1):
            r=euclidean(inital_position[i+1],inital_position[i])
            lamb=link_lengths[i]/r
            inital_position[i]=list(map(add,(1-lamb)*inital_position[i+1],scalarMul(lamb,inital_position[i])))
       #print(b)
        ##Traveling forward

        inital_position[0]=b
        for i in range(inital_position.shape[0]-1):
            r=euclidean(inital_position[i+1],inital_position[i])
            lamb=link_lengths[i]/r
            inital_position[i+1]=list(map(add,(1-lamb)*inital_position[i],scalarMul(lamb,inital_position[i+1])))

        curr_tol=euclidean(goal,inital_position[3])
        itr+=1
        print("iteration: ",itr)
        print(curr_tol)


print(inital_position[3])













