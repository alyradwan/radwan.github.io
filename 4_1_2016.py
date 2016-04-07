from PIL import Image
import numpy as np
import cv2
from redDetect import Crop

mazeWCm=120.5
mazeHCm=80.5
s ='redtest.jpg'
c=Crop().mainCrop(s)
# cv2.imshow('c',c)
s=c
w,h=((Image.open(s)).size)
mappingFactor=(mazeHCm/w+mazeHCm/h)/2
goal=True
queue=[]

def iround(x):
    return int(round(x) - .5) + (x > 0)

def averagePixel(arr):
    averagex = 0
    for i in range(len(arr)):
        averagex = averagex + arr[i][1]
    averagex = averagex / len(arr)
    averagey = 0
    for i in range(len(arr)):
        averagey = averagey + arr[i][0]
    averagey = averagey / len(arr)
    print (averagey, averagex)
    return (averagey, averagex)


# find pixel of specific color
def find(s):
    im = Image.open(s)
    im = im.convert('RGB')
    w, h = im.size
    start = []
    end = []
    # print im.getpixel((3, 498))
    past = im.getpixel((0, 0))

    for j in range(h):
        c = im.getpixel((0, j))
        if past != c:
            start.append((0, j))

        past = c
    for j in range(h):
        c = im.getpixel((w - 1, j))
        if past != c:
            end.append((w - 1, j))

        past = c
    return [averagePixel(start), averagePixel(end)]


def getStartAndEndPoint(s):
    s = cv2.imread(s)
    im = cv2.cvtColor(s, cv2.COLOR_RGB2GRAY)

    # cv2.imshow("Real", s)
    # cv2.waitKey()

def margin(s, (i, j)):
    w, h = s.size
    # [e]=borderAvoid(s,(i,j))
    # print ("dela",iround((22.5/mappingFactor)/2))
    dela=44
    marginleft = i - dela
    if marginleft < 0: marginleft = 0
    marginright = i + dela
    if marginright >= w: marginright = w-1
    margintop = j - dela
    if margintop < 0: margintop = 0
    margindown = j + dela
    if margindown >= h: margindown = h-1

    if s.getpixel((marginleft, j)) != (0, 0, 0) and s.getpixel((marginright, j)) != (0, 0, 0) and s.getpixel(
            (i, margintop)) != (0, 0, 0) and s.getpixel((i, margindown)) != (0, 0, 0)and s.getpixel((marginleft, margindown)) != (0, 0, 0)            and s.getpixel((marginleft, margintop)) != (0, 0, 0):
        return True
    else:
        return False


def build_graph(s):
    # open image
    im = Image.open(s)
    im = im.convert('RGB')
    w, h = im.size

    # graph adjacency list and doors list
    G = {}
    doors = []
    start = []
    # first add nodes to graph and doors
    for j in range(h):
        for i in range(w):
            color = im.getpixel((i, j))
            if color != (0, 0, 0):
                if margin(im, (i, j)):
                    G[(i, j)] = []
                if color != (255, 255, 255):
                    doors.append((i, j))
                if color != (255, 255, 255) and color != (0, 0, 0):
                    start.append((i, j))

    # then add neighbors of each node
    for (i, j) in G.keys():
        if i < w - 1:
            right = (i + 1, j)
            if margin(im,right):
                if im.getpixel(right) != (0, 0, 0):
                    G[(i, j)].append(right)
                    G[right].append((i, j))

        if j < h - 1:
            down = (i, j + 1)
            if margin(im,down):
                if im.getpixel(down) != (0, 0, 0):
                    G[(i, j)].append(down)
                    G[down].append((i, j))

    return [G, doors]


def bfs_mod(graph, start, end):
    # distances dict
    distance = {node: None for node in graph.keys()}
    distance[start] = 0

    # predecessors dict
    parent = {node: None for node in graph.keys()}

    # algorithm
    queue = [start]
    found = False
    while queue and not found:
        node = queue.pop(0)

        for v in graph[node]:
            if v == end:
                found = True
            if distance[v] is None:
                distance[v] = distance[node] + 1
                queue.append(v)
                queue.sort()
                parent[v] = node

    # retrieve shortest path
    path = []
    node = end
    while node is not None:
        path =  [node]+path
        node = parent[node]
    # path.sort()
    return path


def moveangle0(path,point,end):
    goal=True
    i=point+1
    s= path[i][0]

    while True:
        if path[i]==end:
            goal=False
            Distance=path[i][0]-s
            a=[iround(mappingFactor*Distance),0]
            queue.append(a)
            print queue
            break
        elif path[i][1]-path[point][1]!=0 and path[i][0]-path[i-1][0]==0:
            if abs(path[i][1]-path[point][1])>20:
                Distance=path[i][0]-s
                a=[iround(mappingFactor*Distance),0]
                queue.append(a)
                print queue
                break
        i=i+1

    return [i,goal]

def moveanglenN90(path,point,end):
    goal =True
    i=point
    s=i
    point=path[i]
    while True:
        if path[i]==end:
            goal=False
            Distance=path[i][0]-s
            a=[iround(mappingFactor*Distance),-90]
            queue.append(a)
            print queue
            break
        elif path[i][0]-point[0]!=0 and point[1]-path[i][1]>0:
            if abs(path[i][0]-point[0])>10:
                Distance=point[1]-path[i][1]
                Distance=[iround(mappingFactor*Distance),-90]
                queue.append(Distance)
                print queue
                break
        i=i+1
    return [i-4,goal]


def moveangle180(path,point,end):
    goal =True
    i=point[1]
    s=i
    print i
    print path[i][0]
    while True:
        if path[i]==end:
            goal=False
            Distance=path[i][0]-s
            a=[iround(mappingFactor*Distance),180]
            queue.append(a)
            print queue
            break
        elif path[i][1]-point[1]!=0 and path[i][0]-point[0]<5:
            Distance=path[i][1]-s
            Distance=[iround(mappingFactor*Distance),180]
            queue.append(Distance)
            print queue
            break
        point[1]=path[i][1]
        i=i+1
    return [i,goal]


def moveangle90(path,point,end):
    goal=True
    i=point
    s=i
    point=path[i]
    while True:
        if path[i]==end:
            goal=False
            Distance=path[i][0]-s
            a=[iround(mappingFactor*Distance),90]
            queue.append(a)
            print queue
            break
        elif path[i][0]-point[0]!=0 and point[1]-path[i][1]<0:
            if abs(path[i][0]-point[0])>20:
                Distance=abs(point[1]-path[i][1])
                Distance=[iround(mappingFactor*Distance),90]
                queue.append(Distance)
                print queue
                break
        i=i+1

    return [i-4,goal]





def minPath(path,start,end):
    i= path.index(start)
    while goal:

        if path[i+1][0]-path[i][0]!=0 and path[i][1]-path[i+1][1] ==0:
            print "rotate Angel 0.0 "
            [i,g]=moveangle0(path,i,end)
            if g==False:break
        elif path[i+1][0]-path[i][0]==-1 and path[i][1]-path[i+1][1] ==0:
            print "rotate angle -180 "
            [i,g]=moveangle180(path,i,end)
            if g==False:break
        elif path[i+1][0]-path[i][0]==0 and path[i+1][1]-path[i][1] <0:
            print "rotate angle -90 "
            [i,g]=moveanglenN90(path,i,end)
            if g==False:break
        elif path[i+1][0]-path[i][0]==0 and path[i+1][1]-path[i][1] >0:
            print "rotate angle 90"
            [i,g]=moveangle90(path,i,end)
            if g==False:break



def draw_path(path, s):
    im = Image.open(s)
    im = im.convert('RGB')

    for node in path[0:-1]:
        im.putpixel(node, (255, 0, 220))
    im.save('solved_' + s)



G, doors = build_graph(s)

startpos = find(s)
# print startpos
# print startpos[0]
start = startpos[0]
endd = startpos[1]
path = bfs_mod(G, (1,290), (582,275))

# print path
# print path[1][1]
draw_path(path, s)
s = 'solved_'   + s
minPath(path,(1,290),(582,275))
getStartAndEndPoint(s)
