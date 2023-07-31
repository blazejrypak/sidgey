from w1thermsensor import W1ThermSensor
import time
import sqlite3
import paho.mqtt.client as mqtt 
import random
import threading
import RPi.GPIO as GPIO
from sht20 import SHT20

IRRIGATION_SWITCH = 'irrigation-switch'

class DatabaseService():
    def __init__(self) -> None:
        self.db_conn = None
        self.db_cur = None
        self.connect2db()
        self.drop_all_tables()
        self.create_table('measurements')

    def get_db(self):
        return self.db_conn, self.db_cur

    def connect2db(self, db_name='greenhouse.db'):
        self.db_conn = sqlite3.connect(db_name)
        self.db_cur = self.db_conn.cursor()

    def drop_all_tables(self):
        # Get a list of all tables in the database
        self.db_cur.execute("SELECT name FROM sqlite_master WHERE type='table'")
        tables = self.db_cur.fetchall()

        # Loop over each table and drop it
        for table in tables:
            self.db_cur.execute("DROP TABLE IF EXISTS {}".format(table[0]))

        # Commit the changes and close the connection
        self.db_conn.commit()

    def create_table(self, table_name: str):
        # Create the temperature table if it doesn't exist
        self.db_cur.execute(f'''CREATE TABLE IF NOT EXISTS {table_name}
                    (timestamp INTEGER, temperature_in REAL, humidity_in REAL, temperature_out REAL)''')

class TemperaturePublisher():
    def __init__(self) -> None:
        self.mqtt_broker = 'localhost'
        self.mqtt_port = 1883
        self.client_id = f'python-mqtt-{random.randint(0, 1000)}'
        self.mqtt_client = None

        self.connect_mqtt()

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    def connect_mqtt(self):
        self.mqtt_client = mqtt.Client(self.client_id)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port)


    def start_measure_temperature(self):
        db = DatabaseService()
        conn, cur = db.get_db()
        while True:
            sht = SHT20(1, resolution=SHT20.TEMP_RES_14bit)
            humidity_in = sht.read_humid()
            temperature_in = sht.read_temp()
            sensors = W1ThermSensor.get_available_sensors()
            temperature_out = sensors[0].get_temperature()
            timestamp = int(time.time())

            # Insert the temperature data into the database
            cur.execute("INSERT INTO measurements (timestamp, temperature_in, humidity_in, temperature_out) VALUES (?, ?, ?, ?)", (timestamp, temperature_in, humidity_in, temperature_out))
            conn.commit()

            self.mqtt_client.publish("greenhouse/temperature/in", temperature_in)
            self.mqtt_client.publish("greenhouse/humidity/in", humidity_in)
            self.mqtt_client.publish("greenhouse/temperature/out", temperature_out)

            print("Temperature IN: {:.1f}°C, Humidity IN: {:.1f}%, Temperature OUT: {:.1f}°C".format(temperature_in, humidity_in, temperature_out))
            time.sleep(10)

class GPIOController():
    def __init__(self) -> None:
        self.mqtt_broker = 'localhost'
        self.mqtt_port = 1883
        self.client_id = f'python-mqtt-{random.randint(0, 1000)}'
        self.mqtt_client = None
        self.gpio = [
            {'pin': 17, 'state': GPIO.OUT, 'name': 'Irrigation switch', 'device': IRRIGATION_SWITCH}
        ]

        self.init_GPIO()

    def init_GPIO(self):
        GPIO.setmode(GPIO.BCM)
        for gp in self.gpio:
            GPIO.setup(gp['pin'], gp['state'])

    def get_gpio_by_device(self, device):
        for gp in self.gpio:
            if gp['device'] == device:
                return gp['pin']
        return None

    def turn_on_gpio(self, device):
        pin = self.get_gpio_by_device(device)
        if pin:
            GPIO.output(pin, GPIO.HIGH)

    def turn_off_gpio(self, device):
        pin = self.get_gpio_by_device(device)
        if pin:
            GPIO.output(pin, GPIO.LOW)

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    def mqtt_subscriber(self):
        self.mqtt_client = mqtt.Client(self.client_id)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port)
        topics = [
            ('greenhouse/irrigation/switch/setOn', 0),
        ]
        self.mqtt_client.subscribe(topics)
        self.mqtt_client.loop_forever()

    def on_message(self, client: mqtt.Client, userdata: any, msg: mqtt.MQTTMessage):
        topic = msg.topic
        payload = msg.payload.decode('utf-8')
        print(topic, payload)
        if topic == 'greenhouse/irrigation/switch/setOn':
            if payload == '1':
                self.turn_on_gpio(IRRIGATION_SWITCH)
            else:
                self.turn_off_gpio(IRRIGATION_SWITCH)
     

if __name__ == "__main__":
    temperature_publisher_service = TemperaturePublisher()
    GPIO_controller = GPIOController()
    thread1 = threading.Thread(target=temperature_publisher_service.start_measure_temperature)
    thread2 = threading.Thread(target=GPIO_controller.mqtt_subscriber)
    thread1.start()
    thread2.start()
    thread1.join()
    thread2.join()
    GPIO.cleanup()
