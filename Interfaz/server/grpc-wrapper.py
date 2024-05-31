import sys
sys.path.insert(1, '/home/kevin/Desktop/MiniChallenge/Interfaz/protos')

import grpc
from concurrent import futures
import time
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
from std_msgs.msg import Int32
import base64
import numpy as np
import aruco_pb2
import aruco_pb2_grpc

_ONE_DAY_IN_SECONDS = 60 * 60 * 24

# Diccionario para almacenar valores por ID
marker_data = {}

class ArucoDetectorServicer(aruco_pb2_grpc.ArucoDetectorServicer):

    def __init__(self):
        self.bridge = CvBridge()
        self.img_compressed = None
        self.shape = None
        rospy.Subscriber('/aruco_detected_image', Image, self.UpdateImage)
        rospy.Subscriber('/aruco_marker_id', Int32, self.detect_markers_callback)
        print("Initialized gRPC Server")

    def detect_markers_callback(self, msg):
        rospy.loginfo("[INFO] detecting ArUco tags...")
        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        arucoParams = cv2.aruco.DetectorParameters()

        if self.img_compressed is None:
            rospy.logwarn("No image available for detection")
            return

        # Convertir la imagen comprimida de base64 a formato OpenCV
        img_data = base64.b64decode(self.img_compressed)
        np_arr = np.frombuffer(img_data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Detectar marcadores ArUco en la imagen
        corners, ids, rejected = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)

        if ids is not None:
            ids = ids.flatten()
            rospy.loginfo("Detected marker IDs: {}".format(ids))

            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                # Calcular y dibujar el centro del marcador ArUco
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)

                # Guardar los valores en el diccionario para el ID específico
                marker_data[markerID] = {"ID": markerID, "corners": corners.tolist(), "center": [cX, cY]}

            rospy.loginfo("Stored marker data: {}".format(marker_data))
        else:
            rospy.loginfo("No markers detected")

    def DetectMarkers(self, request, context):
        for markerID, data in marker_data.items():
            marker_data_msg = aruco_pb2.ArucoMarkerData(
                id=markerID,
                corners=aruco_pb2.MarkerCorners(corners=[float(coord) for corner in data['corners'] for coord in corner]),
                center=aruco_pb2.MarkerCenter(center=data['center'])
            )
            yield marker_data_msg

    def UpdateImage(self, data):
        try:
            img_original = self.bridge.imgmsg_to_cv2(data)
            self.shape = img_original.shape

            img_compressed = np.array(cv2.imencode('.jpg', img_original)[1]).tobytes()
            self.img_compressed = base64.b64encode(img_compressed).decode('utf-8')
        except Exception as e:
            rospy.logerr("Failed to convert image: {}".format(e))

    def GetImageResult(self, request, context):
        rospy.loginfo("GetImage Got call: " + context.peer())
        results = aruco_pb2.ImageResult()
        if self.img_compressed:
            results.b64img = self.img_compressed
            results.width = self.shape[1]
            results.height = self.shape[0]
        return results

def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    aruco_pb2_grpc.add_ArucoDetectorServicer_to_server(ArucoDetectorServicer(), server)
    server.add_insecure_port('[::]:50051')
    server.start()
    rospy.loginfo("gRPC server started")
    try:
        while not rospy.is_shutdown():
            time.sleep(_ONE_DAY_IN_SECONDS)
    except KeyboardInterrupt:
        server.stop(0)

if __name__ == '__main__':
    rospy.init_node('aruco_detector_node', anonymous=True)
    serve()
