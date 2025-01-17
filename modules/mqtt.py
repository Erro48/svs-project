import time, os
import paho.mqtt.client as mqtt # type: ignore

from modules import utilities
# from modules.utilities import log

# Constants #####################################

FILENAME = os.path.splitext(os.path.basename(__file__))[0]

MQTT_BROKER = "broker.hivemq.com"
MQTT_PORT = 1883
MQTT_TOPIC = "svs-rcta"
MQTT_MESSAGE = "warning: vehicle nearby"

DEFAULT_PUBLISH_INTERVAL = 1 # second

# Functions #####################################

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, reason_code, properties):
    # print(f"Connected with result code {reason_code}")
    utilities.log(f"Connected with result code {reason_code}", FILENAME)
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe(MQTT_TOPIC + "/#")

def on_publish(client, userdata, mid, reason_code, properties):
    # reason_code and properties will only be present in MQTTv5. It's always unset in MQTTv3
    try:
        userdata.remove(mid)
    except KeyError:
        print("on_publish() is called with a mid not present in unacked_publish")
        print("This is due to an unavoidable race-condition:")
        print("* publish() return the mid of the message sent.")
        print("* mid from publish() is added to unacked_publish by the main thread")
        print("* on_publish() is called by the loop_start thread")
        print("While unlikely (because on_publish() will be called after a network round-trip),")
        print(" this is a race-condition that COULD happen")
        print("")
        print("The best solution to avoid race-condition is using the msg_info from publish()")
        print("We could also try using a list of acknowledged mid rather than removing from pending list,")
        print("but remember that mid could be re-used !")

def mqtt_publish(message, topic=MQTT_TOPIC, qos=1, publish_interval=DEFAULT_PUBLISH_INTERVAL):
    global last_message_time
    global unacked_publish
    
    current_time = time.time()
    tdelta = current_time - last_message_time
    if tdelta > publish_interval:
        last_message_time = time.time()
        msg_info = mqttc.publish(topic, message, qos=qos)
        unacked_publish.add(msg_info.mid)

# Setup #########################################

unacked_publish = set()
mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqttc.on_publish = on_publish
mqttc.user_data_set(unacked_publish)

mqttc.on_connect = on_connect

mqttc.connect(MQTT_BROKER, MQTT_PORT, 60)
mqttc.loop_start()

last_message_time = time.time()