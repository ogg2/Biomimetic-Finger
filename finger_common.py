import paho.mqtt.client

# MQTT Topic Names
TOPIC_SET_FINGER_CONFIG = "/finger/set_config"

def client_state_topic(client_id):
    return '/finger/set_config/'.format(client_id)

# MQTT Broker Connection info
MQTT_VERSION = paho.mqtt.client.MQTTv311
MQTT_BROKER_HOST = "localhost"
MQTT_BROKER_PORT = 1883
MQTT_BROKER_KEEP_ALIVE_SECS = 60
