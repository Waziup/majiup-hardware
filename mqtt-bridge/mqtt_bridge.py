#!/usr/bin/env python3

import requests
import json
import threading
import time
import paho.mqtt.client as mqtt
import socket

# Global variable for the TCP connection
tcp_conn = None
tcp_conn_ready = threading.Event()  # Event to signal that the TCP connection is ready

# Wait for localhost API to become available
def wait_for_localhost_api(url="http://localhost/devices", timeout=30):
    print("[ INIT ] Waiting for localhost API to become available...")
    start = time.time()
    while time.time() - start < timeout:
        try:
            r = requests.get(url)
            if r.status_code == 200:
                print("[ INIT ] Localhost API is up.")
                return
        except requests.ConnectionError:
            pass
        time.sleep(1)
    raise Exception("[ INIT ] Timeout waiting for localhost API.")

# Get MQTT topics from the REST API
def get_mqtt_topics():
    try:
        response = requests.get("http://localhost/devices")
        response.raise_for_status()
        tanks = response.json()
    except Exception as e:
        print("Error requesting devices:", e)
        return []

    topics = [f"devices/{tank['id']}/#" for tank in tanks if 'id' in tank]
    return topics

# MQTT connection management
def maintain_mqtt_connection(broker, topic, port, get_tcp_conn):
    client = mqtt.Client()

    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print(f"[ MQTT ] Connected! Subscribing to {topic}")
            client.subscribe(topic)
        else:
            print(f"[ MQTT ] Failed to connect, return code {rc}")

    def on_message(client, userdata, msg):
        print(f"[ MQTT ] Received on {msg.topic}: {msg.payload.decode()}")
        device_id = msg.topic.split("/")[1]
        value = msg.payload.decode()

        tcp_conn_ready.wait()

        tcp = get_tcp_conn()
        try:
            tcp.sendall(f"{device_id},{value}\n".encode())
        except Exception as e:
            print(f"[ TCP ] Error sending message: {e}")
            tcp.close()
            tcp_conn = get_tcp_conn()

    client.on_connect = on_connect
    client.on_message = on_message

    while True:
        if not client.is_connected():
            try:
                print(f"[ MQTT ] Reconnecting to broker for topic {topic}...")
                client.connect(broker, port, 60)
                client.loop_start()
            except Exception as e:
                print(f"[ MQTT ] Connection failed: {e}")
        time.sleep(10)

# TCP server initialization
def init_tcp_server():
    global tcp_conn
    HOST = ''
    PORT = 65432

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    while True:
        try:
            server.bind((HOST, PORT))
            break
        except OSError as e:
            if e.errno == 98:
                print(f"[ TCP ] Address already in use. Retrying...")
                time.sleep(5)
            else:
                raise

    server.listen(1)
    print(f"[ TCP ] Listening for connections on port {PORT}...")

    conn, addr = server.accept()
    print(f"[ TCP ] Connected by {addr}")

    tcp_conn = conn
    tcp_conn_ready.set()
    return conn

# Get TCP connection
def get_tcp_conn():
    global tcp_conn
    if tcp_conn is None:
        print("[ TCP ] Initializing the connection...")
        init_tcp_server()
    return tcp_conn

# Continuously listen for new TCP connections
def listen_for_new_tcp_connections():
    global tcp_conn
    while True:
        try:
            new_conn = init_tcp_server()
            print("[ TCP ] New connection established.")
            tcp_conn = new_conn
            tcp_conn_ready.set()
        except Exception as e:
            print(f"[ TCP ] Error accepting connection: {e}")
        time.sleep(1)

# Main entry point
if __name__ == "__main__":
    wait_for_localhost_api()

    threading.Thread(target=listen_for_new_tcp_connections, daemon=True).start()

    topics = get_mqtt_topics()
    for topic in topics:
        threading.Thread(
            target=maintain_mqtt_connection,
            args=("localhost", topic, 1883, get_tcp_conn),
            daemon=True
        ).start()

    while True:
        try:
            if tcp_conn is None or tcp_conn.fileno() == -1:
                print("[ TCP ] Connection lost or not initialized. Reinitializing...")
                if tcp_conn:
                    tcp_conn.close()
                tcp_conn = None
                init_tcp_server()
        except Exception as e:
            print(f"[ TCP ] Error while checking connection: {e}")
        time.sleep(60)
