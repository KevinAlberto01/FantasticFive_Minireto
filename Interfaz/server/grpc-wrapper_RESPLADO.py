import grpc
from concurrent import futures
import time
import sys
sys.path.insert(1, '/home/kevin/Desktop/MiniChallenge/Interfaz/protos')
import aruco_pb2 #aruco_detector_pb2
import aruco_pb2_grpc #aruco_detector_pb2_grpc
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rospy
from std_msgs.msg import Int32, Float32MultiArray

_ONE_DAY_IN_SECONDS = 60 * 60 * 24

# Diccionario para almacenar valores por ID
marker_data = {}

class ArucoDetectorServicer(aruco_pb2_grpc.ArucoDetectorServicer):

    def __init__(self):
        self.bridge = CvBridge()

    def DetectMarkers(self, request, context):
        while not rospy.is_shutdown():  # Bucle infinito hasta que se detenga el nodo
            # Cargar el diccionario ArUco y los parámetros
            rospy.loginfo("[INFO] detecting ArUco tags...")
            arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            arucoParams = cv2.aruco.DetectorParameters()

            # Cargar la imagen de entrada
            image = cv2.imread("aruco.jpeg")

            # Detectar marcadores ArUco en la imagen
            (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)

            # Verificar si se detectó al menos un marcador ArUco
            if len(corners) > 0:
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

                    # Crear el mensaje de datos del marcador ArUco
                    marker_data_msg = aruco_pb2.ArucoMarkerData(
                        id=markerID,
                        corners=aruco_pb2.MarkerCorners(corners=[float(coord) for corner in corners for coord in corner]),
                        center=aruco_pb2.MarkerCenter(center=[cX, cY])
                    )

                    # Publicar el mensaje del marcador ArUco
                    yield marker_data_msg

                # Imprimir los datos almacenados (opcional)
                rospy.loginfo("Stored marker data: {}".format(marker_data))
                rospy.loginfo("")
            else:
                rospy.loginfo("No markers detected")

            time.sleep(1)  # Esperar un segundo antes de la próxima detección


def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    aruco_pb2_grpc.add_ArucoDetectorServicer_to_server(ArucoDetectorServicer(), server)
    server.add_insecure_port('[::]:50051')
    server.start()
    rospy.loginfo("gRPC server started")
    try:
        while True:
            time.sleep(_ONE_DAY_IN_SECONDS)
    except KeyboardInterrupt:
        server.stop(0)

if __name__ == '__main__':
    rospy.init_node('aruco_detector_node', anonymous=True)
    serve()