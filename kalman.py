import math
import cv2 as cv
import numpy as np
import sys

def trackRobot(frame):
    kernel = np.ones((5,5), np.float32) / 25
    dst = cv.GaussianBlur(frame,(5,5),0)

    dst = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    boundaries = [
        ([30, 125, 126], [179, 255, 255])
    ]

    for (lower, upper) in boundaries:
        mask = cv.inRange(dst, np.array(lower, dtype="uint8"), np.array(upper, dtype="uint8"))
        output = cv.bitwise_and(dst, dst, mask = mask)

    output = cv.cvtColor(output, cv.COLOR_HSV2BGR)
    ret, thresh = cv.threshold(output, 127, 255, 0)

    edged = cv.Canny(thresh, 30, 200)
    #5x5 kernel with full of ones
    kernel = np.ones((3,3),np.uint8)
    dilate = cv.dilate(edged, kernel, iterations = 1)
    contours, hierarchy = cv.findContours(dilate, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    cv.drawContours(image=frame, contours=contours, contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv.LINE_AA)
    cv.imshow("xd", frame)
    for c in contours:
        if cv.arcLength(c, True) > 10:
            M = cv.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        return [int(cX), int(cY)]

def normalize(frame):
    pts1 = np.float32([[360,878],[1903,845],[425, 265], [1466,276]]) # координаты точек на картинке
    pts2 = np.float32([[0, 0], [360, 0], [0, 270], [360, 270]]) # координаты точек в жизни
    # цыганские фокусы с перспективой
    matrix = cv.getPerspectiveTransform(pts1, pts2)
    result = cv.warpPerspective(frame, matrix, (360, 270))
    return cv.flip(rotate_image(result, 180), 1)

def rotate_image(img_origin, angle):
  image_center = tuple(np.array(img_origin.shape[1::-1]) / 2)
  rot_mat = cv.getRotationMatrix2D(image_center, angle, 1.0)
  result = cv.warpAffine(img_origin, rot_mat, img_origin.shape[1::-1], flags=cv.INTER_LINEAR)
  return result

def trackMarkers(frame):
    
    alpha = 2 # контраст
    beta = 20    # яркость

    frame = cv.convertScaleAbs(frame, alpha=alpha, beta=beta)
    cv.imshow("xdBright", frame)
    kernel = np.ones((5,5), np.float32) / 25
    dst = cv.GaussianBlur(frame,(5,5),0)

    dst = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    #output = cv2.cvtColor(output, cv2.COLOR_HSV2BGR)
    ret, thresh = cv.threshold(dst, 127, 255, 0)

    edged = cv.Canny(thresh, 30, 200)
    #5x5 kernel with full of ones
    kernel = np.ones((3,3),np.uint8)
    dilate = cv.dilate(edged, kernel, iterations = 1)
    contours, hierarchy = cv.findContours(dilate, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    cv.drawContours(image=frame, contours=contours, contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv.LINE_AA)
    cv.imshow("xd2", frame)
    markers = []
    for c in contours:
        if float(cv.arcLength(c, True)) > 10 and float(cv.arcLength(c, True)) < 40:
            M = cv.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            markers.append([cX, cY])

    return markers

def straightLines(frame, x1, y1, x2, y2, x3, y3):
    aX = math.fabs(x2 - x3)
    aY = math.fabs(y2 - y3)
    bX = math.fabs(x1 - x3)
    bY = math.fabs(y1 - y3)
    cX = math.fabs(x2 - x1)
    cY = math.fabs(y2 - y1)

    a = math.sqrt(aX ** 2 + aY ** 2)
    b = math.sqrt(bX ** 2 + bY ** 2)
    c = math.sqrt(cX ** 2 + cY ** 2)
    # a^2 = b^2 + c^2 - 2 * b * c * cos(a)

    print(str(x1) + '; ' + str(y1) + '; ' + str(x2) + '; ' + str(y2) + '; ' + str(x3) + '; ' + str(y3))

    frame = normalize(frame)
    cv.line(frame, [x1, y1], [x2, y2], (255, 0, 0), 2)
    cv.line(frame, [x1, y1], [x3, y3], (0, 255, 0), 2)   
    #cv.imshow("hui3", frame) 
    return math.degrees(math.acos((((-a ** 2 + b ** 2 + c ** 2) / (2 * b * c)))))


# Instantiate OCV kalman filter
class KalmanFilter:

    kf = cv.KalmanFilter(4, 2)
    kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
    kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)

    def Estimate(self, coordX, coordY):
        ''' This function estimates the position of the object'''
        measured = np.array([[np.float32(coordX)], [np.float32(coordY)]])
        self.kf.correct(measured)
        predicted = self.kf.predict()
        return predicted



#Performs required image processing to get ball coordinated in the video
class ProcessImage:

    def DetectObject(self, markerIndex):

        vid = cv.VideoCapture(0)

        if(vid.isOpened() == False):
            print('Cannot open input video')
            return

        vid = normalize(vid)
        width = int(vid.get(3))
        height = int(vid.get(4))

        # Create Kalman Filter Object
        kfObj = KalmanFilter()
        predictedCoords = np.zeros((2, 1), np.float32)

        while(vid.isOpened()):
            rc, frame = vid.read()

            if(rc == True):
                [ballX, ballY] = trackRobot(frame)
                predictedCoords = kfObj.Estimate(ballX, ballY)
                [markerX, markerY] = trackMarkers[markerIndex]
                print(straightLines(frame, ballX, ballY, predictedCoords[0], predictedCoords[1], markerX, markerY))

                # Draw Actual coords from segmentation
                #cv.circle(frame, (int(ballX), int(ballY)), 20, [0,0,255], 2, 8)
                #cv.line(frame,(int(ballX), int(ballY + 20)), (int(ballX + 50), int(ballY + 20)), [100,100,255], 2,8)
                #cv.putText(frame, "Actual", (int(ballX + 50), int(ballY + 20)), cv.FONT_HERSHEY_SIMPLEX,0.5, [50,200,250])

                # Draw Kalman Filter Predicted output
                #cv.circle(frame, (int(predictedCoords[0]), int(predictedCoords[1])), 20, [0,255,255], 2, 8)
                #cv.line(frame, (int(predictedCoords[0]) + 16, int(predictedCoords[1]) - 15), (int(predictedCoords[0]) + 50, int(predictedCoords[1]) - 30), [100, 10, 255], 2, 8)
                #cv.putText(frame, "Predicted", (int(predictedCoords[0] + 50), int(predictedCoords[1] - 30)), cv.FONT_HERSHEY_SIMPLEX, 0.5, [50, 200, 250])
                cv.imshow('Input', frame)

                if (cv.waitKey(300) & 0xFF == ord('q')):
                    break

            else:
                break

        vid.release()
        cv.destroyAllWindows()

    # Segment the green ball in a given frame
    def DetectBall(self, frame):

        # Set threshold to filter only green color & Filter it
        lowerBound = np.array([130,30,0], dtype = "uint8")
        upperBound = np.array([255,255,90], dtype = "uint8")
        greenMask = cv.inRange(frame, lowerBound, upperBound)

        # Dilate
        kernel = np.ones((5, 5), np.uint8)
        greenMaskDilated = cv.dilate(greenMask, kernel)
        #cv.imshow('Thresholded', greenMaskDilated)

        # Find ball blob as it is the biggest green object in the frame
        [nLabels, labels, stats, centroids] = cv.connectedComponentsWithStats(greenMaskDilated, 8, cv.CV_32S)

        # First biggest contour is image border always, Remove it
        stats = np.delete(stats, (0), axis = 0)
        try:
            maxBlobIdx_i, maxBlobIdx_j = np.unravel_index(stats.argmax(), stats.shape)

        # This is our ball coords that needs to be tracked
            ballX = stats[maxBlobIdx_i, 0] + (stats[maxBlobIdx_i, 2]/2)
            ballY = stats[maxBlobIdx_i, 1] + (stats[maxBlobIdx_i, 3]/2)
            return [ballX, ballY]
        except:
               pass

        return [0,0]


#Main Function
def main():

    processImg = ProcessImage()
    processImg.DetectObject()


if __name__ == "__main__":
    main()

print('Program Completed!')