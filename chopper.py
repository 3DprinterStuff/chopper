import numpy
from stl import mesh

def generateGcode(filename):
        your_mesh = mesh.Mesh.from_file(filename)

        out = open(filename+".gcode","w")

        rotationalAxis = [0,5,5]

        floorTriangles = []
        lineMap = {}

        def walkTriangleFull(triangle):
            out.write("\n")
            out.write("# "+str(triangle)+"\n")
            startPoint = triangle[0]
            leftPoint = triangle[1]
            rightPoint = triangle[2]

            # align triangle
            if leftPoint[0] < startPoint[0]:
                tmp = startPoint
                startPoint = leftPoint
                leftPoint = tmp
            if rightPoint[0] < startPoint[0]:
                tmp = startPoint
                startPoint = rightPoint
                rightPoint = tmp
            if rightPoint[1] < leftPoint[1]:
                tmp = rightPoint
                rightPoint = leftPoint
                leftPoint = tmp
            
            out.write("# "+str(startPoint)+" -> "+str(leftPoint)+"\n")
            out.write("# "+str(startPoint)+" -> "+str(rightPoint)+"\n")

            out.write("G1 X%d Y%d Z%d \n"%(startPoint[0],startPoint[1],startPoint[2]))
            out.write("G1 X%d Y%d Z%d \n"%(leftPoint[0],leftPoint[1],leftPoint[2]))
            out.write("G1 X%d Y%d Z%d \n"%(rightPoint[0],rightPoint[1],rightPoint[2]))
            out.write("G1 X%d Y%d Z%d \n"%(startPoint[0],startPoint[1],startPoint[2]))

            leftLine = (startPoint,leftPoint)
            rightLine = (startPoint,rightPoint)


            # draw lines between left and right border till done
            xPosition = startPoint[0]
            updown = False

            while xPosition < leftPoint[0] and xPosition < rightPoint[0]:
                xPosition += 0.5

                xPercentageLeft = (xPosition-startPoint[0])/(leftPoint[0]-startPoint[0])
                yPositionLeft = (leftPoint[1]-startPoint[1])*xPercentageLeft+startPoint[1]

                xPercentageRight = (xPosition-startPoint[0])/(rightPoint[0]-startPoint[0])
                yPositionRight = (rightPoint[1]-startPoint[1])*xPercentageRight+startPoint[1]

                if updown:
                    out.write("G1 X%f Y%f Z%f \n"%(xPosition,yPositionRight,startPoint[2]))
                    out.write("G1 X%f Y%f Z%f \n"%(xPosition,yPositionLeft,startPoint[2]))
                    updown = False
                else:
                    out.write("G1 X%f Y%f Z%f \n"%(xPosition,yPositionLeft,startPoint[2]))
                    out.write("G1 X%f Y%f Z%f \n"%(xPosition,yPositionRight,startPoint[2]))
                    updown = True

            while xPosition < leftPoint[0] or xPosition < rightPoint[0]:
                if leftPoint[0] < rightPoint[0]:
                    xPosition += 0.5

                    xPercentageBack = (xPosition-leftPoint[0])/(rightPoint[0]-leftPoint[0])
                    yPositionBack = (rightPoint[1]-leftPoint[1])*xPercentageBack+leftPoint[1]

                    xPercentageRight = (xPosition-startPoint[0])/(rightPoint[0]-startPoint[0])
                    yPositionRight = (rightPoint[1]-startPoint[1])*xPercentageRight+startPoint[1]

                    if updown:
                        out.write("G1 X%f Y%f Z%f \n"%(xPosition,yPositionBack,startPoint[2]))
                        out.write("G1 X%f Y%f Z%f \n"%(xPosition,yPositionRight,startPoint[2]))
                        updown = False
                    else:
                        out.write("G1 X%f Y%f Z%f \n"%(xPosition,yPositionRight,startPoint[2]))
                        out.write("G1 X%f Y%f Z%f \n"%(xPosition,yPositionBack,startPoint[2]))
                        updown = True

        '''
        slice by Z layers
        '''
        zHeight = 0
        while 1:
            done = True
            counter = 0
            maxLength = your_mesh.points.size/9
            while (counter < maxLength):
                points = [tuple(your_mesh.points[counter][0:3]),tuple(your_mesh.points[counter][3:6]),tuple(your_mesh.points[counter][6:9])]

                # get triangles 
                levelPoints = 0
                lowerPoints = []
                for point in points:
                    if point[2] < zHeight+0.2 and point[2] >= zHeight:
                        levelPoints += 1
                    elif point[2] < zHeight:
                        lowerPoints.append(point)

                if levelPoints == 3:
                    walkTriangleFull(points)
                    done = False

                if lowerPoints and len(lowerPoints) < 3:
                    done = False
                    if len(lowerPoints) > 1:
                        top = None
                        for point in points:
                            if not point in lowerPoints:
                                top = point

                        zPercentageStart = (zHeight-lowerPoints[0][2])/(top[2]-lowerPoints[0][2])
                        xPositionStart = lowerPoints[0][0]+(top[0]-lowerPoints[0][0])*zPercentageStart
                        yPositionStart = lowerPoints[0][1]+(top[1]-lowerPoints[0][1])*zPercentageStart

                        zPercentageEnd = (zHeight-lowerPoints[1][2])/(top[2]-lowerPoints[1][2])
                        xPositionEnd = lowerPoints[1][0]+(top[0]-lowerPoints[1][0])*zPercentageEnd
                        yPositionEnd = lowerPoints[1][1]+(top[1]-lowerPoints[1][1])*zPercentageEnd

                        out.write("G1 X%f Y%f Z%f\n" % (xPositionStart,yPositionStart,zHeight))
                        out.write("G1 X%f Y%f Z%f\n\n" % (xPositionEnd,yPositionEnd,zHeight))
                            
                    else:
                        lower = None
                        higherPoints = []
                        for point in points:
                            if point in lowerPoints:
                                lower = point
                            else:
                                higherPoints.append(point)

                        zPercentageStart = (zHeight-higherPoints[0][2])/(lower[2]-higherPoints[0][2])
                        xPositionStart = higherPoints[0][0]+(lower[0]-higherPoints[0][0])*zPercentageStart
                        yPositionStart = higherPoints[0][1]+(lower[1]-higherPoints[0][1])*zPercentageStart

                        zPercentageEnd = (zHeight-higherPoints[1][2])/(lower[2]-higherPoints[1][2])
                        xPositionEnd = higherPoints[1][0]+(lower[0]-higherPoints[1][0])*zPercentageEnd
                        yPositionEnd = higherPoints[1][1]+(lower[1]-higherPoints[1][1])*zPercentageEnd

                        out.write("G1 X%f Y%f Z%f\n" % (xPositionStart,yPositionStart,zHeight))
                        out.write("G1 X%f Y%f Z%f\n\n" % (xPositionEnd,yPositionEnd,zHeight))
                            

                """
                out.write("\n")
                out.write("G1 X%d Y%d Z%d\n" % (your_mesh.points[counter][0:3][0],your_mesh.points[counter][0:3][1],your_mesh.points[counter][0:3][2]))
                out.write("G1 X%d Y%d Z%d\n" % (your_mesh.points[counter][3:6][0],your_mesh.points[counter][3:6][1],your_mesh.points[counter][3:6][2]))
                out.write("G1 X%d Y%d Z%d\n" % (your_mesh.points[counter][6:9][0],your_mesh.points[counter][6:9][1],your_mesh.points[counter][6:9][2]))
                """
                counter += 1

            zHeight += 0.2
            if done:
                break

        out.close()
