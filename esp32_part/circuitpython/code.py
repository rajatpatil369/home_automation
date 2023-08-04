import board
import touchio
import busio
import time

import ssl
import socketpool
import wifi
import adafruit_minimqtt.adafruit_minimqtt as MQTT

stat = [0, 0, 0, 0]
prev_scan = [1, 1, 1, 1]
mat = (touchio.TouchIn(board.D13), touchio.TouchIn(board.D12), touchio.TouchIn(board.D14), touchio.TouchIn(board.D27))
u2 = busio.UART(board.TX2, board.RX2, baudrate=9600)
colors = (0x9ACD32, 0x40E0D0, 0xDC143C, 0x808000, 0xFF6347, 0x9932CC, 0x6A5ACD, 0x483D8B, 0xA0522D, 0xDB7093, 0xEE82EE, 0x6B8E23, 0x2E8B57, 0xFF00FF, 0xD8BFD8, 0xD2B48C, 0xFF4500, 0x1E90FF, 0x228B22, 0x663399, 0x00FF7F, 0xFA8072, 0x8B4513, 0x800080, 0x6A5ACD, 0x008080)
i = 0
l = len(colors)
monitored_parameters = [0, 0, 0, 0, 0, 0, 1]
# pir_motion, ldr_lux, dht_temp, dht_humi, uss_cm, mq2_lpg_ppm, ir_position

# wifi.radio.connected
# wifi.radio.connect('Nokia 1', '10:41 M 26-dec-22')
# (wifi.radio.ipv4_address, wifi.radio.ipv4_subnet, wifi.radio.ipv4_gateway)
pool = socketpool.SocketPool(wifi.radio)
MQTT_BROKER = '192.168.43.31'
MQTT_USERNAME = 'esp32'
MQTT_PASSWORD = 'password'
assert wifi.radio.connected, 'Network connection error.'
mqtt_client = MQTT.MQTT(
    broker=MQTT_BROKER,
    username=MQTT_USERNAME,
    password=MQTT_PASSWORD,
    client_id='client_id_ESP32',    # add here
    socket_pool=pool,
    ssl_context=ssl.create_default_context(),
)
def connect(mqtt_client, userdata, flags, rc):
    print(f'Connected to MQTT Broker!\nFlags: {flags}\n RC: {rc}')
def disconnect(mqtt_client, userdata, rc):
    print(f'Disconnected from MQTT Broker!')
def subscribe(mqtt_client, userdata, topic, granted_qos):
    print(f'Subscribed to {topic} with QOS level {granted_qos}')
def unsubscribe(mqtt_client, userdata, topic, pid):
    print(f'Unsubscribed from {topic} with PID {pid}')
def publish(mqtt_client, userdata, topic, pid):
    print(f'Published to {topic} with PID {pid}')
def message(client, topic, message):
    print(f'New message on topic {topic}: {message}')
    # 'has/manual', 'has/user_settings'
    u2.write(message.encode())
mqtt_client.on_connect = connect
mqtt_client.on_disconnect = disconnect
mqtt_client.on_subscribe = subscribe
mqtt_client.on_unsubscribe = unsubscribe
mqtt_client.on_publish = publish
mqtt_client.on_message = message
mqtt_client.connect()
mqtt_client.subscribe('has/manual')
mqtt_client.subscribe('has/user_settings')

def read_inputs():
    global prev_scan, i
    scan = list(map(lambda tp: 1 if tp.value else 0, mat))
    if any(scan) and scan != prev_scan:
        if scan[0]: stat[0] ^= 1
        if scan[1]: stat[1] ^= 1
        stat[2] = scan[2] and monitored_parameters[-1]
        stat[3] = scan[3]
        msg = b':~' + bytearray([    # change
            (stat[0] << 7) | (stat[1] << 5) | (stat[2] << 2),    # two bits 6th and 5th for window blind
            (stat[3] << 7),    # remaining 7 bits are unused...
            (0xff0000 & colors[i]) >> 16,
            (0x00ff00 & colors[i]) >> 8,
            (0x0000ff & colors[i]),
        ]) + bytearray([    # mask
            (scan[0] << 7) | (scan[1] << 6) | (scan[2] << 3),
            scan[3] << 7,
            10  # '\n'
        ])
        _=u2.write(msg)
        print(':change=', *[f'{j:08b}' for j in msg[2:4]])
        print(f'color= {colors[i]} -> {colors[i]:06X}', *[f'{j:02X}={j:3}' for j in msg[4:7]], sep=' | ')
        print('mask=', f'{msg[7]:08b}', f'{msg[8]:08b}', end='\n\n')
        if scan[3]:
            i += 1
            if i == l: i = 0
    prev_scan = scan


while True:
    if u2.in_waiting:
        msg = u2.readline()
        if msg[0] != b':': continue
        monitored_parameters = [float(i) if b'.' in i else int(i) for i in msg[1:].strip().split()]
        # mqtt_client.publish('has/monitored_parameters', msg)
    read_inputs()
    # u2.write(b':!1 52 106 26.2 33.7 34.6 15 125 200\n')    # settings

    time.sleep(0.1)
