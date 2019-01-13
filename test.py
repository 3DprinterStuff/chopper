from zipfile import ZipFile
import xml.dom.minidom
import math

'''
classes
'''

class Dispatcher(object):
    def __init__(self,extrusionFactor=0.075,filename="cube.gcode",baseTemperature=210,baseSpeed=4000.0):
        self.extrusionFactor = extrusionFactor
        self.outFile = open(filename,"w") 

        self.commitGcode("""

M127
G1 Z5 F5000 ; lift nozzle
M104 S%f ; set extruder temp
M140 S0 ; set bed temp
M109 S%f ; wait for extruder temp
G21 ; set units to millimeters
M83 ; use relative distances for extrusion

M73 P0
G1 Z0.400 F%f
M103 ; extruder off

M101 ; extruder on

G1 Z1.000 F%f
        """%(baseTemperature,baseTemperature,baseSpeed,baseSpeed))

    def addLine(self,point1,point2,extraExtrusion=1.0):
        extrusionFactor = self.extrusionFactor*extraExtrusion
        distance = math.sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2+(point1[2]-point2[2])**2)
        self.commitGcode("G1 X%f Y%f Z%f E0"%(point1[0],point1[1],point1[2]))
        self.commitGcode("G1 X%f Y%f Z%f E%f"%(point2[0],point2[1],point2[2],distance*extrusionFactor))

    def commitGcode(self,gcode):
        self.outFile.write(gcode+"\n")

    def tearDown(self):
        self.commitGcode("""
M103 ; extruder off
M127
M104 S0 ; turn off temperature
G1 Z100  ;
M84     ; disable motors
""")

        self.outFile.close()

class FileReader(object):
    def __init__(self,filename="cube.amf"):
        self.filename = filename

    def getRawData(self,filename):
        input_zip=ZipFile(filename)
        rawXml = input_zip.read(input_zip.namelist()[0])

        DOMTree = xml.dom.minidom.parseString(rawXml)
        collection = DOMTree.documentElement
        objects = collection.getElementsByTagName("object")

        return objects

    def extractVertices(self,objects):
        vertices = []
        for child in objects[0].getElementsByTagName("vertices")[0].childNodes:
            if (child.nodeType in (child.TEXT_NODE, child.CDATA_SECTION_NODE)):
                continue
            x = float(child.getElementsByTagName("x")[0].childNodes[0].nodeValue)
            y = float(child.getElementsByTagName("y")[0].childNodes[0].nodeValue)
            z = float(child.getElementsByTagName("z")[0].childNodes[0].nodeValue)

            vertices.append((x,y,z))
        return vertices

    def extractTriangles(self,objects):
        triangles = []
        for child in objects[0].getElementsByTagName("volume")[0].childNodes:
            if (child.nodeType in (child.TEXT_NODE, child.CDATA_SECTION_NODE)):
                continue
            v1 = int(child.getElementsByTagName("v1")[0].childNodes[0].nodeValue)
            v2 = int(child.getElementsByTagName("v2")[0].childNodes[0].nodeValue)
            v3 = int(child.getElementsByTagName("v3")[0].childNodes[0].nodeValue)

            triangles.append((v1,v2,v3))
        return triangles

'''
functions
'''
def buildVertexMap(triangles):
    verticesToTriangles = {}
    for triangle in triangles:
        if not triangle[0] in verticesToTriangles:
            verticesToTriangles[triangle[0]] = []
        verticesToTriangles[triangle[0]].append(triangle)

        if not triangle[1] in verticesToTriangles:
            verticesToTriangles[triangle[1]] = []
        verticesToTriangles[triangle[1]].append(triangle)

        if not triangle[2] in verticesToTriangles:
            verticesToTriangles[triangle[2]] = []
        verticesToTriangles[triangle[2]].append(triangle)
    return verticesToTriangles

def getBottomConvexPolys(convexPolys,vertices,zCutOff=0):
    bottomConvexPolys = []
    for convexPoly in convexPolys:
        bottomVertices = []
        for vertex in convexPoly:
            if vertices[vertex][2] > 0.2+zCutOff:
                continue
            if vertices[vertex][2] < zCutOff:
                continue
            bottomVertices.append(vertex)
    
        if len(bottomVertices) < len(convexPoly):
            continue
        bottomConvexPolys.append(convexPoly)
    return bottomConvexPolys

def calculateNeighbours(triangles):
    neighbours = {}
    for triangle in triangles:
        for subTriangle in triangles:
            sharedVertices = []
            for vertex in triangle:
                if not vertex in subTriangle:
                    continue
                sharedVertices.append(vertex)

            if len(sharedVertices) == 2: 
                if not triangle in neighbours:
                    neighbours[triangle] = []
                neighbours[triangle].append(subTriangle)
    return neighbours

def calculateNormal(triangle):
    u = (triangle[1][0]-triangle[0][0],
         triangle[1][1]-triangle[0][1],
         triangle[1][2]-triangle[0][2])
    v = (triangle[2][0]-triangle[0][0],
         triangle[2][1]-triangle[0][1],
         triangle[2][2]-triangle[0][2])
    n = ( u[1]*v[2] - u[2]*v[1],
          u[2]*v[0] - u[0]*v[2],
          u[0]*v[1] - u[1]*v[0])
    return n

def isNormalEquivalent(normal1,normal2):
    counter = 0
    significantIndex = None
    while counter < len(normal1):
        if not normal1[counter] == 0.0:
            significantIndex = counter
            break
        counter += 1

    if significantIndex == None:
        return False

    if normal2[significantIndex] == 0.0:
        return False

    scale = normal1[significantIndex]/normal2[significantIndex]

    counter = 0
    while counter < len(normal1):
        if not normal1[counter] == normal2[counter]*scale:
            return False
        counter += 1
    return True

def expandVertex(vertex,vertices):
    return vertices[vertex]

def expandTriangle(abstractTriangle,vertices):
    return (expandVertex(abstractTriangle[0],vertices),
            expandVertex(abstractTriangle[1],vertices),
            expandVertex(abstractTriangle[2],vertices))

def mergePolys(poly1,poly2):
    commonVertices = []
    poly1 = list(poly1)
    poly2 = list(poly2)

    # get common verices
    for vertex in poly1:
        for compareVertex in poly2:
            if vertex == compareVertex:
                commonVertices.append(vertex)
    if not len(commonVertices) == 2:
        raise Exception("wrong number of vertices")

    # resort polys 
    while not (poly1[0] in commonVertices and poly1[-1] in commonVertices):
        poly1.append(poly1.pop(0))
    while not (poly2[0] in commonVertices and poly2[-1] in commonVertices):
        poly2.append(poly2.pop(0))

    # ensure oposing rotation for polys
    if poly1[0] == poly2[0]:
        poly2 = list(reversed(poly2))

    # merge polys
    newPoly = poly1
    newPoly.extend(poly2[1:-1])

    return newPoly

def reducePolys(inputPolys,vertices,neighbours):
    polys = []
    inputBuckets = inputPolys[:]
    while inputBuckets:
        poly = inputBuckets.pop()
        normal = calculateNormal(expandTriangle(poly,vertices))
        newPoly = None
        for comparePoly in neighbours[poly]:
            compareNormal = calculateNormal(expandTriangle(comparePoly,vertices))
            if isNormalEquivalent(normal,compareNormal):
                newPoly = mergePolys(poly,comparePoly)
                inputBuckets.remove(comparePoly)

                #inputBuckets.append(newPoly)
                if newPoly:
                    polys.append(newPoly)

                break
    
        if newPoly == None:
            polys.append(poly)
    return polys

def getUpperLeftVertex(poly,vertices):
    upperLeftPoly = None
    for vertex in poly:
        if upperLeftPoly == None:
            upperLeftPoly = vertex
            continue
        if vertices[upperLeftPoly][0] > vertices[vertex][0] or (vertices[upperLeftPoly][0] == vertices[vertex][0] and vertices[upperLeftPoly][1] < vertices[vertex][1]):
            upperLeftPoly = vertex
    
    counter = 0
    startIndex = None
    polyLen = len(poly)
    while counter < len(poly):
       if poly[counter] == upperLeftPoly:
           startIndex = counter
       counter += 1
    
    return startIndex

def getLowerRightVertex(poly,vertices):
    upperLeftPoly = None
    for vertex in poly:
        if upperLeftPoly == None:
            upperLeftPoly = vertex
            continue
        if vertices[upperLeftPoly][0] < vertices[vertex][0] or (vertices[upperLeftPoly][0] == vertices[vertex][0] and vertices[upperLeftPoly][1] > vertices[vertex][1]):
            upperLeftPoly = vertex
    
    counter = 0
    startIndex = None
    polyLen = len(poly)
    while counter < len(poly):
       if poly[counter] == upperLeftPoly:
           startIndex = counter
       counter += 1
    
    return startIndex

def calculateOutlinePath(poly,vertices,startIndex = 0):
    outlinePath = []
    currentIndex = startIndex
    polyLen = len(poly)
    while currentIndex < polyLen:
        outlinePath.append(vertices[poly[currentIndex]])
        currentIndex += 1
    currentIndex = 0
    while currentIndex <= startIndex:
        outlinePath.append(vertices[poly[currentIndex]])
        currentIndex += 1
    
    return outlinePath

def printPoly(poly,vertices,dispatcher,stepSize = 0.4, axis="x",extraExtrusion=1.0,infillExtraExtrusion=1.0):
    print("#",poly)
    if axis == "x":
        startIndex = getUpperLeftVertex(poly,vertices)
    elif axis == "y":
        startIndex = getLowerRightVertex(poly,vertices)
        
    outlinePath = calculateOutlinePath(poly,vertices,startIndex)
    print("#",outlinePath)
    print("# printing outline")

    lastPoint = outlinePath[0]
    for point in outlinePath[1:]:
        dispatcher.addLine(lastPoint,point,extraExtrusion=extraExtrusion)
        lastPoint = point
    
    normal = calculateNormal((outlinePath[startIndex],outlinePath[startIndex+1],outlinePath[startIndex-1]))
    if normal[2] > 0:
        outlinePath = list(reversed(outlinePath))
        startIndex = len(outlinePath)-startIndex

    print("# printing %s fill with %d stepSize"%(axis,stepSize))
    toFill = outlinePath[:]
    if axis == "x":
        axisIndex = 0
    elif axis == "y":
        axisIndex = 1
    position = toFill[0][axisIndex]
    upDown = True
    while len(toFill) > 2:
        position += stepSize

        # remove obsolete vertexes
        while toFill[1][axisIndex] < position+(stepSize*0.5):
            toFill.pop(0)
            if len(toFill) < 2:
                break
        if len(toFill) < 2:
            break
        while toFill[-2][axisIndex] < position+(stepSize*0.5):
            toFill.pop()
            if len(toFill) < 2:
                break
        if len(toFill) < 2:
            break

        # get intersections
        upperLine = (toFill[0],toFill[1])
        percentage = (position-upperLine[0][axisIndex])/(upperLine[1][axisIndex]-upperLine[0][axisIndex])
        xPositionTop = (upperLine[1][0]-upperLine[0][0])*percentage+upperLine[0][0]
        yPositionTop = (upperLine[1][1]-upperLine[0][1])*percentage+upperLine[0][1]-0.2
        zPositionTop = (upperLine[1][2]-upperLine[0][2])*percentage+upperLine[0][2]
        lowerLine = (toFill[-1],toFill[-2])
        percentage = (position-lowerLine[0][axisIndex])/(lowerLine[1][axisIndex]-lowerLine[0][axisIndex])
        xPositionBottom = (lowerLine[1][0]-lowerLine[0][0])*percentage+lowerLine[0][0]
        yPositionBottom = (lowerLine[1][1]-lowerLine[0][1])*percentage+lowerLine[0][1]+0.2
        zPositionBottom = (lowerLine[1][2]-lowerLine[0][2])*percentage+lowerLine[0][2]

        if yPositionTop-yPositionBottom < 0.2 and yPositionTop-yPositionBottom > -0.2:
           continue

        if upDown:
            dispatcher.addLine((xPositionTop,yPositionTop,zPositionTop),(xPositionBottom,yPositionBottom,zPositionBottom),extraExtrusion=extraExtrusion*infillExtraExtrusion)
            upDown = False
        else:
            dispatcher.addLine((xPositionBottom,yPositionBottom,zPositionBottom),(xPositionTop,yPositionTop,zPositionTop),extraExtrusion=extraExtrusion*infillExtraExtrusion)
            upDown = True

def sliceZLayer(polys,vertices,dispatcher,zCutoff,zHeight):
    bottomConvexPolys = getBottomConvexPolys(polys,vertices,zCutoff)
    #print("bottomConvexPolys")
    #print(bottomConvexPolys)

    firstTop = True
    for poly in bottomConvexPolys:
        if zCutoff < 0.5 or zCutoff > 9.5:
            if firstTop == True:
                printPoly(poly,vertices,dispatcher,extraExtrusion=0.7)
                firstTop = False
            else:
                printPoly(poly,vertices,dispatcher)
        else:
            printPoly(poly,vertices,dispatcher,stepSize=4,axis="x",extraExtrusion=0.5,infillExtraExtrusion=2.0)
            printPoly(poly,vertices,dispatcher,stepSize=4,axis="y",extraExtrusion=0.5,infillExtraExtrusion=2.0)
        """
        elif zCutoff%1 > 0.5:
            printPoly(poly,stepSize=4,axis="x",extraExtrusion=1.0)
        else:
            printPoly(poly,stepSize=4,axis="y",extraExtrusion=1.0)
        """

        polys.remove(poly) 
    
    intersectingPolys = []
    for poly in polys:
        hasLower = False
        for vertex in poly:
            if vertices[vertex][2] <= zCutoff:
                hasLower = True
            
        if hasLower:
            intersectingPolys.append(poly)

    movedVertices = {}
    for poly in intersectingPolys:
        state = "findLower"
        counter = 0
        for vertex in poly:
            if state == "findLower":
                if vertices[vertex][2] <= zCutoff+zHeight:
                    state = "findLastLower"
            if state == "findLastLower":
                if vertices[vertex][2] > zCutoff+zHeight:
                    break
            counter += 1
        startIndex = counter-1
        positionedPolyOutline = calculateOutlinePath(poly,vertices,startIndex)

        if positionedPolyOutline[1][2] < zCutoff+zHeight*2:
            polys.remove(poly)
            break

        line = (positionedPolyOutline[0],positionedPolyOutline[1])
        percentage = ((zCutoff+zHeight)-line[0][2])/(line[1][2]-line[0][2])
        xPosition = (line[1][0]-line[0][0])*percentage+line[0][0]
        yPosition = (line[1][1]-line[0][1])*percentage+line[0][1]
        zPosition = (line[1][2]-line[0][2])*percentage+line[0][2]
        leftVertex = (xPosition,yPosition,zPosition)

        lastVertex = None
        for vertex in reversed(positionedPolyOutline):
            if vertex[2] > zCutoff+zHeight:
                break
            lastVertex = vertex
        line = (lastVertex,vertex)
        percentage = ((zCutoff+zHeight)-line[0][2])/(line[1][2]-line[0][2])
        xPosition = (line[1][0]-line[0][0])*percentage+line[0][0]
        yPosition = (line[1][1]-line[0][1])*percentage+line[0][1]
        zPosition = (line[1][2]-line[0][2])*percentage+line[0][2]
        rightVertex = (xPosition,yPosition,zPosition)
        #print("G1 X%f Y%f Z%f"%(leftVertex[0],leftVertex[1],leftVertex[2]))
        #print("G1 X%f Y%f Z%f"%(rightVertex[0],rightVertex[1],rightVertex[2]))

        newPolyOutline = [leftVertex]
        for vertex in positionedPolyOutline:
            if vertex[2] < zCutoff+zHeight*2:
                continue
            newPolyOutline.append(vertex)
        newPolyOutline.append(rightVertex)

        vertices.append(leftVertex)
        leftVertexIndex = len(vertices)-1
        vertices.append(rightVertex)
        rightVertexIndex = len(vertices)-1

        newPoly = []
        newPoly.append(leftVertexIndex)
        steps = len(newPolyOutline)-2
        index = startIndex+1
        stopIndex = None
        polyLen = len(poly)
        while index < polyLen and steps:
            newPoly.append(poly[index])
            stopIndex = index
            index += 1
            steps -= 1
        index = 0
        while index < startIndex and steps:
            newPoly.append(poly[index])
            stopIndex = index
            index += 1
            steps -= 1
        newPoly.append(rightVertexIndex)

        movedVertices[poly[startIndex]] = leftVertexIndex
        movedVertices[poly[stopIndex]] = rightVertexIndex

        polys.remove(poly)
        polys.append(newPoly)
    
    for poly in bottomConvexPolys:
        counter = 0
        while counter < len(poly):
           if poly[counter] in movedVertices:
              poly[counter] = movedVertices[poly[counter]]
           counter += 1

        polys.append(poly)

def slicePolys(polys,vertices,dispatcher): 
    zCutoff = 0
    zHeight = 0.2
    while polys:
        print("slicing layer %f"%(zCutoff))
        sliceZLayer(polys,vertices,dispatcher,zCutoff,zHeight)
        zCutoff += zHeight

'''
dirty code
'''
def slice(baseFilename):
    dispatcher = Dispatcher(filename=baseFilename+".code")
    fileReader = FileReader()

    #objects = fileReader.getRawData("convexPolyPolstered.amf")
    objects = fileReader.getRawData(baseFilename+".amf")

    vertices = fileReader.extractVertices(objects)
    #print("vertices")
    #print(vertices)

    triangles = fileReader.extractTriangles(objects)
    #print("triangles")
    #print(triangles)

    verticesToTriangles = buildVertexMap(triangles)
    #print("verticesToTriangles")
    #print(verticesToTriangles)

    neighbours = calculateNeighbours(triangles)
    #print("neighbours")
    #print(neighbours)

    polys = reducePolys(triangles,vertices,neighbours)
    #print("polys")
    #print(polys)
                    
    slicePolys(polys,vertices,dispatcher)

    dispatcher.tearDown()

slice("cube")
