from flask import Flask, jsonify, render_template
import threading
import time
import requests
import json
import schedule
import sys
import grpc
from concurrent import futures
sys.path.insert(1, '/home/kevin/Desktop/MiniChallenge/Interfaz/protos')
import aruco_pb2 
import aruco_pb2_grpc 

app = Flask(__name__)
app.marker_data = {}  # Inicializa marker_data como un diccionario vacío
result = None
result_img = None

def save_to_json(data):
    try:
        with open('marker_data.json', 'w') as f:
            json.dump(data, f)
        print("Datos guardados en marker_data.json correctamente.")
    except Exception as e:
        print("Error al guardar los datos en marker_data.json:", e)

def call_api():
    global app
    url = 'http://127.0.0.1:8042/restgatewaydemo/getmultcoords'
    try:
        response = requests.post(url, json={})
        response.raise_for_status()
        result = response.json()
        values = result.get("values")
        if values:
            values.pop()
            app.marker_data.update(result)  # Actualiza marker_data con los datos obtenidos
            save_to_json(app.marker_data)  # Guarda los datos en un archivo JSON
            print("Datos de la API REST actualizados correctamente.")
        else:
            print("Error: 'values' key not found in the JSON response")
    except requests.RequestException as e:
        print("Error haciendo la solicitud a la API:", e)

def call_api_image():
    global app
    url = 'http://127.0.0.1:8042/restgatewaydemo/getimageresult'
    try:
        response = requests.post(url, json={})
        response.raise_for_status()
        result_img = response.json()
        app.marker_data.update(result_img)  # Actualiza marker_data con los datos de imagen obtenidos
        save_to_json(app.marker_data)  # Guarda los datos en un archivo JSON
        print("Datos de imagen de la API REST actualizados correctamente.")
    except requests.RequestException as e:
        print("Error haciendo la solicitud a la API de imagen:", e)

def call_grpc_api():
    global app
    try:
        channel = grpc.insecure_channel('localhost:50051')
        stub = aruco_pb2_grpc.ArucoDetectorStub(channel)
        empty_request = aruco_pb2.Empty()
        for marker_data in stub.DetectMarkers(empty_request):
            app.marker_data[marker_data.id] = {
                "id": marker_data.id,
                "corners": marker_data.corners.corners,
                "center": marker_data.center.center
            }
            save_to_json(app.marker_data)  # Guarda los datos en un archivo JSON
            print("Datos del servidor gRPC actualizados correctamente.")
    except grpc.RpcError as e:
        print("Error llamando al servidor gRPC:", e)

if __name__ == '__main__':
    schedule.every(0.1).seconds.do(call_api)
    schedule.every(0.1).seconds.do(call_api_image)

    t = threading.Thread(target=call_grpc_api)
    t.start()

    @app.route('/api/marker_data')
    def get_marker_data():
        global app
        return jsonify(app.marker_data)

    @app.route('/api/image')
    def get_image():
        global result_img
        if result_img is not None:
            return "data:image/jpg;base64, " + result_img["b64img"].replace('"','')
        else:
            return "Not found"
        
    @app.route('/')
    def index():
        global app
        return render_template('imagen.html', marker_data=app.marker_data)

    app.run(debug=True, port=8002)
