from zipfile import ZipFile
import xml.dom.minidom

def getRawData(filename):
    input_zip=ZipFile(filename)
    rawXml = input_zip.read(input_zip.namelist()[0])

    DOMTree = xml.dom.minidom.parseString(rawXml)
    collection = DOMTree.documentElement
    objects = collection.getElementsByTagName("object")

    return objects

def extractVertices(objects):
    vertices = []
    for child in objects[0].getElementsByTagName("vertices")[0].childNodes:
        if (child.nodeType in (child.TEXT_NODE, child.CDATA_SECTION_NODE)):
            continue
        x = float(child.getElementsByTagName("x")[0].childNodes[0].nodeValue)
        y = float(child.getElementsByTagName("y")[0].childNodes[0].nodeValue)
        z = float(child.getElementsByTagName("z")[0].childNodes[0].nodeValue)

        vertices.append((x,y,z))
    return vertices

def etractTriangles(objects):
    triangles = []
    for child in objects[0].getElementsByTagName("volume")[0].childNodes:
        if (child.nodeType in (child.TEXT_NODE, child.CDATA_SECTION_NODE)):
            continue
        v1 = int(child.getElementsByTagName("v1")[0].childNodes[0].nodeValue)
        v2 = int(child.getElementsByTagName("v2")[0].childNodes[0].nodeValue)
        v3 = int(child.getElementsByTagName("v3")[0].childNodes[0].nodeValue)

        triangles.append((v1,v2,v3))
    return triangles

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

def getBottomConvexPolys(convexPolys):
    bottomConvexPolys = []
    for convexPoly in convexPolys:
        bottomVertices = []
        for vertex in convexPoly:
            if vertices[vertex][2] > 0.2:
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

def reducePolys(inputPolys):
    polys = []
    inputBuckets = triangles[:]
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

def getUpperLeftVertex(poly):
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

def calculateOutlinePath(poly,startIndex):
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

def printPoly(poly):
    print("#",poly)
    startIndex = getUpperLeftVertex(poly)
    outlinePath = calculateOutlinePath(poly,startIndex)
    print("#",outlinePath)
    print("# printing outline")

    for point in outlinePath:
        print("G1 X%f Y%f Z%f"%(point[0],point[1],point[2]))

    print("# printing fill")
    toFill = outlinePath[:]
    xPosition = toFill[0][0]
    upDown = True
    while len(toFill) > 2:
        xPosition += 0.2

        # remove obsolete vertexes
        while toFill[1][0] < xPosition:
            toFill.pop(0)
            if len(toFill) < 2:
                break
        if len(toFill) < 2:
            break
        while toFill[-2][0] < xPosition:
            toFill.pop()
            if len(toFill) < 2:
                break
        if len(toFill) < 2:
            break

        # get intersections
        upperLine = (toFill[0],toFill[1])
        percentage = (xPosition-upperLine[0][0])/(upperLine[1][0]-upperLine[0][0])
        xPositionTop = (upperLine[1][0]-upperLine[0][0])*percentage+upperLine[0][0]
        yPositionTop = (upperLine[1][1]-upperLine[0][1])*percentage+upperLine[0][1]-0.2
        zPositionTop = (upperLine[1][2]-upperLine[0][2])*percentage+upperLine[0][2]
        lowerLine = (toFill[-1],toFill[-2])
        percentage = (xPosition-lowerLine[0][0])/(lowerLine[1][0]-lowerLine[0][0])
        xPositionBottom = (lowerLine[1][0]-lowerLine[0][0])*percentage+lowerLine[0][0]
        yPositionBottom = (lowerLine[1][1]-lowerLine[0][1])*percentage+lowerLine[0][1]+0.2
        zPositionBottom = (lowerLine[1][2]-lowerLine[0][2])*percentage+lowerLine[0][2]

        if yPositionTop-yPositionBottom < 0.2:
           continue

        if upDown:
            print("G1 X%f Y%f Z%f"%(xPositionTop,yPositionTop,zPositionTop))
            print("G1 X%f Y%f Z%f"%(xPositionBottom,yPositionBottom,zPositionBottom))
            upDown = False
        else:
            print("G1 X%f Y%f Z%f"%(xPositionBottom,yPositionBottom,zPositionBottom))
            print("G1 X%f Y%f Z%f"%(xPositionTop,yPositionTop,zPositionTop))
            upDown = True

#objects = getRawData("convexPolyPolstered.amf")
objects = getRawData("cube.amf.zip")

vertices = extractVertices(objects)
#print("vertices")
#print(vertices)

triangles = etractTriangles(objects)
#print("triangles")
#print(triangles)

verticesToTriangles = buildVertexMap(triangles)
#print("verticesToTriangles")
#print(verticesToTriangles)

neighbours = calculateNeighbours(triangles)
#print("neighbours")
#print(neighbours)

polys = reducePolys(triangles)
#print("polys")
#print(polys)

bottomConvexPolys = getBottomConvexPolys(polys)
#print("bottomConvexPolys")
#print(bottomConvexPolys)

for poly in bottomConvexPolys:
    printPoly(poly)
    























