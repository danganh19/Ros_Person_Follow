from imageai.Detection import ObjectDetection
import os
import cv2


#os.system("clear")
execution_path = os.getcwd()
print(execution_path)
detector = ObjectDetection()
print(detector)
detector.setModelTypeAsYOLOv3()
custom_objects = detector.CustomObjects(person=True) 
detector.setModelPath(os.path.join(execution_path , "yolo.h5"))
detector.loadModel(detection_speed="fastest") 

vs = cv2.VideoCapture(0)
while True:
    check, frame = vs.read()
    cv2.imwrite("webCamImage.jpg", frame)
    detections = detector.detectCustomObjectsFromImage(custom_objects=custom_objects, input_image=os.path.join(execution_path , "webCamImage.jpg"), output_image_path=os.path.join(execution_path , "newImage.jpg"), minimum_percentage_probability=20)
    print("DA", detections)
    if len(detections) != 0:
        x1 = detections[0]["box_points"][0]
        y1 = detections[0]["box_points"][1]
        x2 = detections[0]["box_points"][2]
        y2 = detections[0]["box_points"][3]
        print("--------------------------------")
    else:
        print("Not found any person")
        print("--------------------------------")
    img = cv2.imread('newImage.jpg')
    cv2.imshow('frame', img)
    key = cv2.waitKey(1)
    if key == ord("q"):
            break
    
    # break

cv2.destroyAllWindows()
vs.stop()
