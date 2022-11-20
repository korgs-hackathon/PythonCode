import random
import time
import paho.mqtt.client as mqtt

broker = '10.0.2.2'
port = 1883
topic = "korgs/topic"
cliend_id = f'pyhton-mqtt-{random.randint(0, 1000)}'

def connect():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connection Successful")
        else:
            print("Failed to connect")
    client = mqtt_client.Client(client_id)
    #client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

def publish(client):
    msg_count = 0
    while True:
        time.sleep(1)
        msg = f"messages: {msg_count}"
        result = client.publish(topic, msg)
        status = result[0]
        if status == 0:
            print(f"Sent '{msg}' to topic '{topic}'")
        else:
            print("Failed to send")
        msg_count+=1

        
