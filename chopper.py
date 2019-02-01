from zipfile import ZipFile
import xml.dom.minidom
import math

'''
classes
'''

class Dispatcher(object):
    def __init__(self,extrusionFactor=0.03,filename="cube.gcode",baseTemperature=210,baseSpeed=4000.0):
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
                print("!!!!!!!!!!")
                print(vertex)
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
    print(neighbours)
    while inputBuckets:
        poly = inputBuckets.pop()
        normal = calculateNormal(expandTriangle(poly,vertices))
        newPoly = None
        for comparePoly in neighbours[poly]:
            if not comparePoly in inputBuckets:
                continue
            compareNormal = calculateNormal(expandTriangle(comparePoly,vertices))
            if isNormalEquivalent(normal,compareNormal):
                newPoly = tuple(mergePolys(poly,comparePoly))
                inputBuckets.remove(comparePoly)

                #inputBuckets.append(newPoly)
                if newPoly:
                    inputBuckets.append(newPoly)
                    neighbours[newPoly] = []
                    neighbours[newPoly].extend(neighbours[poly])
                    neighbours[newPoly].extend(neighbours[comparePoly])

                break
    
        if newPoly == None:
            polys.append(list(poly))
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

def getLowerLeftVertex(poly,vertices):
    lowerLeftPoly = None
    for vertex in poly:
        if lowerLeftPoly == None:
            lowerLeftPoly = vertex
            continue
        if vertices[lowerLeftPoly][1] > vertices[vertex][1] or (vertices[lowerLeftPoly][1] == vertices[vertex][1] and vertices[lowerLeftPoly][0] > vertices[vertex][0]):
            lowerLeftPoly = vertex
    
    counter = 0
    startIndex = None
    polyLen = len(poly)
    while counter < len(poly):
       if poly[counter] == lowerLeftPoly:
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

def printPoly(poly,vertices,dispatcher,stepSize = 0.4, axis="x",extraExtrusion=1.0,infillExtraExtrusion=1.0,innerLines=[]):
    print("#",poly)
    if axis == "x":
        startIndex = getUpperLeftVertex(poly,vertices)
    elif axis == "y":
        startIndex = getLowerLeftVertex(poly,vertices)
        
    outlinePath = calculateOutlinePath(poly,vertices,startIndex)
    print("#",outlinePath)
    dispatcher.commitGcode("# printing outline")

    reverseMap = {}
    for vertex in poly:
       reverseMap[vertices[vertex]] = vertex

    lastPoint = outlinePath[0]
    for point in outlinePath[1:]:
        if not (reverseMap[point],reverseMap[lastPoint]) in innerLines:
            dispatcher.addLine(lastPoint,point,extraExtrusion=extraExtrusion)
            pass
        lastPoint = point
    
    normal = calculateNormal((outlinePath[startIndex],outlinePath[startIndex+1],outlinePath[startIndex-1]))
    if normal[2] > 0:
        outlinePath = list(reversed(outlinePath))
        startIndex = len(outlinePath)-startIndex

    dispatcher.commitGcode("# printing %s fill with %f stepSize"%(axis,stepSize))
    toFill = outlinePath[:]
    if axis == "x":
        axisIndex = 0
    elif axis == "y":
        axisIndex = 1
    position = toFill[0][axisIndex]
    dispatcher.commitGcode("# position "+str(position))
    upDown = True
    position -= stepSize
    while len(toFill) > 2:
        dispatcher.commitGcode("# toFill "+str(toFill))
        position += stepSize

        # remove obsolete vertexes
        while toFill[1][axisIndex] < position:
            toFill.pop(0)
            if len(toFill) < 2:
                break
        if len(toFill) < 2:
            break
        while toFill[-2][axisIndex] < position:
            toFill.pop()
            if len(toFill) < 2:
                break
        if len(toFill) < 2:
            break

        # get intersections
        upperLine = (toFill[0],toFill[1])
        percentage = (position-upperLine[0][axisIndex])/(upperLine[1][axisIndex]-upperLine[0][axisIndex])
        topPosition = []
        for axis in (0,1,2):
            topPosition.append((upperLine[1][axis]-upperLine[0][axis])*percentage+upperLine[0][axis])
        """
        if axisIndex == 0:
            topPosition[1] -= 0.2
        elif axisIndex == 1:
            topPosition[0] += 0.2
        """

        lowerLine = (toFill[-1],toFill[-2])
        percentage = (position-lowerLine[0][axisIndex])/(lowerLine[1][axisIndex]-lowerLine[0][axisIndex])
        bottomPosition = []
        for axis in (0,1,2):
            bottomPosition.append((lowerLine[1][axis]-lowerLine[0][axis])*percentage+lowerLine[0][axis])
        """
        if axisIndex == 0:
            bottomPosition[1] += 0.2
        elif axisIndex == 1:
            bottomPosition[0] -= 0.2
        """

        dispatcher.commitGcode("# top %s bottom %s "%(topPosition,bottomPosition))

        """
        if axisIndex == 0:
            if topPosition[1] < bottomPosition[1]:
                continue
        elif axisIndex == 1:
            if topPosition[0] > bottomPosition[0]:
                continue
        """

        if axisIndex == 0:
            dispatcher.addLine(bottomPosition,topPosition,extraExtrusion=extraExtrusion*infillExtraExtrusion)
            dispatcher.addLine(topPosition,bottomPosition,extraExtrusion=extraExtrusion*infillExtraExtrusion)
        if axisIndex == 1:
            dispatcher.addLine(topPosition,bottomPosition,extraExtrusion=extraExtrusion*infillExtraExtrusion)
            dispatcher.addLine(bottomPosition,topPosition,extraExtrusion=extraExtrusion*infillExtraExtrusion)

    if axisIndex == 1:
        dispatcher.addLine(outlinePath[0],outlinePath[0],extraExtrusion=0)
        pass
    if axisIndex == 0:
        dispatcher.addLine(outlinePath[3],outlinePath[3],extraExtrusion=0)
        pass

def sliceZLayer(polys,vertices,dispatcher,zCutoff,zHeight,axis):
    bottomConvexPolys = getBottomConvexPolys(polys,vertices,zCutoff)

    firstTop = True
    lines = []
    innerLines = []
    for poly in bottomConvexPolys:
        lastVertex = poly[-1]
        for vertex in poly:
            lines.append((lastVertex,vertex))
            if (vertex,lastVertex) in lines and not (vertex,lastVertex) in innerLines:
                innerLines.append((lastVertex,vertex))
                innerLines.append((vertex,lastVertex))
            lastVertex = vertex

    maxHeight = 0
    for vertex in vertices:
        if maxHeight < vertex[2]:
            maxHeight = vertex[2]

    for poly in bottomConvexPolys:
        dispatcher.commitGcode("#poly"+str(poly))
        for vertex in poly:
            dispatcher.commitGcode("# "+str(vertices[vertex]))
        if 1==0 and (zCutoff < 0.5 or zCutoff > maxHeight-0.5):
            if firstTop == True:
                printPoly(poly,vertices,dispatcher,extraExtrusion=0.7,axis=axis)
                firstTop = False
            else:
                printPoly(poly,vertices,dispatcher,axis=axis)
        else:
            printPoly(poly,vertices,dispatcher,stepSize=5,axis="x",extraExtrusion=0.5,infillExtraExtrusion=2.0,innerLines=innerLines)
            printPoly(poly,vertices,dispatcher,stepSize=5,axis="y",extraExtrusion=0.5,infillExtraExtrusion=2.0,innerLines=innerLines)

        polys.remove(poly) 
    
    intersectingPolys = []
    for poly in polys:
        hasLower = False
        for vertex in poly:
            if vertices[vertex][2] <= zCutoff+zHeight:
                hasLower = True
            
        if hasLower:
            intersectingPolys.append(poly)

    movedVertices = {}
    movedLines = {}
    for poly in intersectingPolys:
        print("cut Poly",poly)
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

        if not line in movedLines:
            vertices.append(leftVertex)
            leftVertexIndex = len(vertices)-1
            movedLines[line] = leftVertexIndex
            movedLines[tuple(reversed(line))] = leftVertexIndex
        else:
            leftVertexIndex = movedLines[line]

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

        if not line in movedLines:
            vertices.append(rightVertex)
            rightVertexIndex = len(vertices)-1
            movedLines[line] = rightVertexIndex
            movedLines[tuple(reversed(line))] = rightVertexIndex
        else:
            rightVertexIndex = movedLines[line]

        #print("G1 X%f Y%f Z%f"%(leftVertex[0],leftVertex[1],leftVertex[2]))
        #print("G1 X%f Y%f Z%f"%(rightVertex[0],rightVertex[1],rightVertex[2]))

        newPolyOutline = [leftVertex]
        for vertex in positionedPolyOutline:
            if vertex[2] < zCutoff+zHeight*2:
                continue
            newPolyOutline.append(vertex)
        newPolyOutline.append(rightVertex)

        print(newPolyOutline)
        
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
        while index < startIndex+1 and steps:
            newPoly.append(poly[index])
            stopIndex = index
            index += 1
            steps -= 1
        newPoly.append(rightVertexIndex)

        if not poly[startIndex] in movedVertices:
            movedVertices[poly[startIndex]] = []
        movedVertices[poly[startIndex]].append(leftVertexIndex)
        if not poly[stopIndex] in movedVertices:
            movedVertices[poly[stopIndex]] = []
        movedVertices[poly[stopIndex]].append(rightVertexIndex)

        print("removing: ",poly)
        print("adding: ",newPoly)

        polys.remove(poly)
        polys.append(newPoly)

    print("movedVertices")
    print(movedVertices)
    
    # build mapping of poly bordering on vertices
    vertexMap = {}
    for poly in bottomConvexPolys:
        for vertex in movedVertices:
            if not vertex in poly:
                continue
            if not vertex in vertexMap:
                vertexMap[vertex] = []
            vertexMap[vertex].append((poly,poly[:]))

    for poly in bottomConvexPolys:
        print("moving",poly)
        print(movedVertices)
        counter = 0

        while counter < len(poly):
           origVertexId = poly[counter]
           if not origVertexId in movedVertices:
               counter += 1
               continue
           # move the vertex
           if len(movedVertices[origVertexId]) == 1:
               poly[counter] = movedVertices[origVertexId][0]
           else:
               # get leftmost poly
               leftMostPoly = None
               for searchPoly in vertexMap[origVertexId]:
                   searchIndex = searchPoly[1].index(origVertexId)
                   prevVertex = searchPoly[1][searchIndex-1]
                   found = False
                   for comparePoly in vertexMap[origVertexId]:
                       if comparePoly == searchPoly:
                           continue
                       if not prevVertex in comparePoly[1]:
                           continue
                       found = True

                   if not found:
                       leftMostPoly = searchPoly[0]
                       break

               # add vertices to lefmost poly
               leftMostPoly.remove(origVertexId)
               leftMostPoly.extend(movedVertices[origVertexId])
               checkIndex = -1
               while checkIndex < len(leftMostPoly)-1:
                   direction = calculateNormal([vertices[leftMostPoly[checkIndex-1]],vertices[leftMostPoly[checkIndex]],vertices[leftMostPoly[checkIndex+1]]])
                   if direction[2] > 0.01:
                       tmp = leftMostPoly[checkIndex-1]
                       leftMostPoly[checkIndex-1] = leftMostPoly[checkIndex]
                       leftMostPoly[checkIndex] = tmp
                       checkIndex = -1
                   else:
                       checkIndex += 1

               # find last added vertex
               state = "findFirst"
               index = 0
               for vertex in leftMostPoly:
                   if state == "findFirst":
                       if not vertex in movedVertices[origVertexId]:
                           index += 1
                       else:
                           state = "findLast"
                   if state == "findLast":
                       if vertex in movedVertices[origVertexId]:
                           index += 1
                       else:
                           break
               index -= 1

               # modify vertex map
               movedVertices[origVertexId] = [leftMostPoly[index]]

               if not leftMostPoly == poly:
                   poly[counter] = movedVertices[origVertexId][0]
               else:
                   counter = 0
                   continue
               
           counter += 1

        polys.append(poly)
        print("readding",poly)

    '''
    print("------------------------")
    print(movedVertices)
    for poly in bottomConvexPolys:
        foundUpper = False
        for vertexId in poly:
            vertex = vertices[vertexId]

            if vertex[2] > zCutoff:
                foundUpper = True
    
        if foundUpper:
            print("moving slow polys")
            for vertexId in poly:
                vertex = vertices[vertexId]

                if vertex[2] >= zCutoff and vertex[2] < zCutoff+zHeight:
                    vertices[vertexId] = (vertex[0],vertex[1],zCutoff+zHeight)
                    print(vertexId)
                    print(vertices[vertexId])
    '''

def slicePolys(polys,vertices,dispatcher): 
    zCutoff = 0
    zHeight = 0.2
    axis = "x"
    while polys:
        print("slicing layer %f"%(zCutoff))
        sliceZLayer(polys,vertices,dispatcher,zCutoff,zHeight,axis)
        print(polys)
        zCutoff += zHeight

        if axis == "x":
            axis = "y"
        else:
            axis = "x"


'''
dirty code
'''
def slice(baseFilename):
    dispatcher = Dispatcher(filename=baseFilename+".gcode")
    fileReader = FileReader()

    objects = fileReader.getRawData(baseFilename+".amf")

    vertices = fileReader.extractVertices(objects)
    #print("vertices")
    #print(vertices)

    triangles = fileReader.extractTriangles(objects)
    print("triangles")
    print(triangles)

    #dispatcher.commitGcode(";triangles")
    #for poly in triangles:
    #    dispatcher.commitGcode("#"+str(poly))
    #    for vertexId in list(poly)+[poly[0]]:
    #        vertex = vertices[vertexId]
    #        dispatcher.commitGcode("G1 X%f Y%f Z%f"%(vertex[0],vertex[1],vertex[2]))
                    

    verticesToTriangles = buildVertexMap(triangles)
    #print("verticesToTriangles")
    #print(verticesToTriangles)

    neighbours = calculateNeighbours(triangles)
    #print("neighbours")
    #print(neighbours)

    polys = reducePolys(triangles,vertices,neighbours)
    #polys = []
    #for poly in triangles:
    #    polys.append(list(poly))
    print("polys")
    print(polys)

    #dispatcher.commitGcode(";polys")
    #for poly in polys:
    #    dispatcher.commitGcode("#"+str(poly))
    #    for vertexId in list(poly)+[poly[0]]:
    #        vertex = vertices[vertexId]
    #        dispatcher.commitGcode("G1 X%f Y%f Z%f"%(vertex[0],vertex[1],vertex[2]))
                    
    slicePolys(polys,vertices,dispatcher)

    dispatcher.tearDown()
