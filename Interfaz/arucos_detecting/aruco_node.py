#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Int32, Float32MultiArray
import sys
sys.path.insert(1, '/home/kevin/Desktop/MiniChallenge/Interfaz/protos')
import argparse
import grpc
import base64  # Asegúrate de importar base64
import aruco_pb2
import aruco_pb2_grpc

# Diccionario para almacenar valores por ID
marker_data = {}

def main():
    rospy.init_node('aruco_detector_node', anonymous=True)
    bridge = CvBridge()

    # Crear publicadores para la imagen y el ID del marcador
    image_pub = rospy.Publisher('/aruco_detected_image', Image, queue_size=10)
    id_pub = rospy.Publisher('/aruco_marker_id', Int32, queue_size=10)
    corners_pub = rospy.Publisher('/corners', Float32MultiArray, queue_size=10)
    center_pub = rospy.Publisher('/center', Float32MultiArray, queue_size=10)

    # Analizador de argumentos
    ap = argparse.ArgumentParser()
    ap.add_argument("-t", "--type", type=str, default="DICT_4X4_50", help="type of ArUCo tag to detect")
    ap.add_argument("-i", "--image", type=str, default="aruco.jpeg", help="path to input image containing ArUco tags")
    args = vars(ap.parse_args())

    # Diccionario de tipos de marcadores ArUco soportados
    ARUCO_DICT = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        # Otros tipos de diccionarios...
    }

    # Verificar si el tipo de marcador ArUco es soportado
    if ARUCO_DICT.get(args["type"], None) is None:
        rospy.loginfo("[INFO] ArUCo tag of '{}' is not supported".format(args["type"]))
        sys.exit(0)

    # Conexión gRPC
    channel = grpc.insecure_channel('localhost:50051')
    stub = aruco_pb2_grpc.ArucoDetectorStub(channel)

    while not rospy.is_shutdown():  # Bucle infinito hasta que se detenga el nodo
        # Cargar el diccionario ArUco y los parámetros
        rospy.loginfo("[INFO] detecting '{}' tags...".format(args["type"]))
        arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[args["type"]])
        arucoParams = cv2.aruco.DetectorParameters()

        # Cargar la imagen de entrada
        image = cv2.imread(args["image"])
        if image is None:
            rospy.loginfo("[ERROR] Failed to load image at path: {}".format(args["image"]))
            continue

        # Detectar marcadores ArUco en la imagen
        (corners, ids, rejected) = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)

        # Verificar si se detectó al menos un marcador ArUco
        if len(corners) > 0:
            ids = ids.flatten()
            rospy.loginfo("Detected marker IDs: {}".format(ids))

            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                # Convertir cada par de coordenadas (x, y) a enteros
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                # Dibujar el cuadro delimitador del marcador ArUco
                cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

                # Calcular y dibujar el centro del marcador ArUco
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

                # Dibujar el ID del marcador ArUco en la imagen
                cv2.putText(image, str(markerID),
                            (topLeft[0], topLeft[1] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 2)

                # Guardar los valores en el diccionario para el ID específico
                marker_data[markerID] = {"ID": markerID, "corners": corners, "center": (cX, cY)}

                # Publicar el ID del marcador
                id_msg = Int32()
                id_msg.data = markerID
                id_pub.publish(id_msg)

                # Publicar las esquinas del marcador
                corners_msg = Float32MultiArray()
                corners_msg.data = corners.flatten().tolist()
                corners_pub.publish(corners_msg)

                # Publicar el centro del marcador
                center_msg = Float32MultiArray()
                center_msg.data = [cX, cY]
                center_pub.publish(center_msg)

            # Imprimir los datos almacenados (opcional)
            rospy.loginfo("Stored marker data: {}".format(marker_data))
            rospy.loginfo("")
        else:
            rospy.loginfo("No markers detected")

        # Convertir la imagen a un mensaje ROS y publicarla
        ros_image = bridge.cv2_to_imgmsg(image, "bgr8")
        image_pub.publish(ros_image)

        # Mostrar la imagen de salida
        cv2.imshow("Image", image)
        cv2.waitKey(1)
        # cv2.imwrite("/home/kevin/Desktop/MiniChallenge/Interfaz/client/templates/Arucos_detected.jpg", image)

        # Llamar al servicio gRPC para obtener el resultado de la imagen
        try:
            response = stub.GetImageResult(aruco_pb2.Empty())
            with open("/home/kevin/Desktop/MiniChallenge/Interfaz/client/templates/Arucos_detected.jpg", "wb") as f:
                f.write(base64.b64decode(response.b64img))
        except grpc.RpcError as e:
            rospy.loginfo("[ERROR] Error llamando a GetImageResult(): {}".format(e))

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
