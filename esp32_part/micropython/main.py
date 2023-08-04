from machine import TouchPad, Pin, UART
from time import sleep, time

# a = Pin(0, Pin.IN)
# 
# def read(u):
#     while a():
#         if u.any():
#             x=u.readline()
#             _=u.write(x)
#             print(x)
#         sleep(0.25)

stat = [0, 0, 0, 0]
mat = [TouchPad(Pin(13)), TouchPad(Pin(12)), TouchPad(Pin(14)), TouchPad(Pin(27))]    # Returns a smaller number when touched
u2 = UART(2, baudrate=9600, tx=17, rx=16)
colors = (0x9ACD32, 0x40E0D0, 0xDC143C, 0x808000, 0xFF6347, 0x9932CC, 0x6A5ACD, 0x483D8B, 0xA0522D, 0xDB7093, 0xEE82EE, 0x6B8E23, 0x2E8B57, 0xFF00FF, 0xD8BFD8, 0xD2B48C, 0xFF4500, 0x1E90FF, 0x228B22, 0x663399, 0x00FF7F, 0xFA8072, 0x8B4513, 0x800080, 0x6A5ACD, 0x008080)
i = 0
l = len(colors)

while True:
    if any(scan := list(map(lambda x: 1 if x.read() < 100 else 0, mat))):
        if scan[0]: stat[0] ^= 1
        if scan[1]: stat[1] ^= 1
        stat[2] = scan[2]
        stat[3] = scan[3]
        msg = b':' + bytearray([    # change
            (stat[0] << 7) | (stat[1] << 5) | (stat[2] << 2),    # two bits 6th and 5th for window blind
            (stat[3] << 7),    # remaining 7 bits are unused...
            (0xff0000 & colors[i]) >> 16,
            (0x00ff00 & colors[i]) >> 8,
            (0x0000ff & colors[i]),
        ]) + bytearray([    # mask
            (scan[0] << 7) | (scan[1] << 6) | (scan[2] << 3),
            scan[3] << 7
        ]) + b':1 52 106 26.2 33.7 34.6 15 125 200\n'    # settings
        _=u2.write(msg)
        print(':change=', *[f'{j:08b}' for j in msg[:3]])
        print(f'color= {colors[i]} -> {colors[i]:06X}', *[f'{j:02X}={j:3}' for j in msg[3:6]], sep=' | ')
        print('mask=', f'{msg[6]:08b}', f'{msg[7]:08b}')
        print(':settings=', f'{msg[8:]}', end='\n\n')
        if scan[3]:
            i += 1
            if i == l: i = 0
    while scan == list(map(lambda x: 1 if x.read() < 100 else 0, mat)):
        sleep(0.2)

'''
https://mosquitto.org/documentation/using-the-snap
https://mosquitto.org/documentation/authentication-methods

note: snap can be used to install mosquitto (and clients) on linux debian bullseye, but when the mosquitto_passwd tool is used, the following error appears (even if used withs udo):
Error: Unable to open file /etc/mosquitto/password_file for writing. Permission denied.

https://mosquitto.org/blog/2013/01/mosquitto-debian-repository
https://mosquitto.org/download

hence:
cd ~/Downloads/
wget http://repo.mosquitto.org/debian/mosquitto-repo.gpg.key

sudo apt-key add mosquitto-repo.gpg.key
Warning: apt-key is deprecated. Manage keyring files in trusted.gpg.d instead (see apt-key(8)).
OK

rm mosquitto-repo.gpg.key
cd /etc/apt/sources.list.d/
sudo wget http://repo.mosquitto.org/debian/mosquitto-bullseye.list
apt update
sudo apt install mosquitto mosquitto-clients
cd /etc/mosquitto/
sudo mv mosquitto.conf _mosquitto.conf

sudo vi mosquitto.conf
listener 1883    # default
allow_anonymous false
password_file /etc/mosquitto/password_file

sudo mosquitto_passwd -c /etc/mosquitto/password_file <user_name>    # username e.g. esp32, type in and confirm the password
ps aux | grep mosquitto    # note the PID in 2nd column
sudo kill -HUP <mosquitto_pid>    # reload mosquitto with new configurations
sudo systemctl restart mosquitto.service

note: for finding the broker IP use:
hostname -I | awk '{ print $1 }'

mosquitto_sub -d -h localhost -p 1883 -t has/ -u esp32 -P hello,\ world! 
mosquitto_pub -v -h localhost -t 'has/' -m 'Hello, world!' -u esp32 -P hello,\ world! 
'''


from umqtt.simple import MQTTClient
MQTTClient('rajat', '192.168.43.31', 1883, 'esp32', 'hello, world!').connect()

MQTT_BROKER = '192.168.43.31'
MQTT_PORT = 1883
MQTT_TOPIC = 'has/'

MQTT_USERNAME = 'esp32'
MQTT_PASSWORD = 'hello, world!'

client_id = 'esp32_client'
client = MQTTClient(client_id, MQTT_BROKER, MQTT_PORT, MQTT_USERNAME, MQTT_PASSWORD)
client.connect()
print('Connected to MQTT broker')


def mqtt_callback(topic, msg):
    print(f'Received message: Topic={topic}, Message={msg}')

client.set_callback(mqtt_callback)

client.subscribe(MQTT_TOPIC)
print('Subscribed to topic:', MQTT_TOPIC)

while True:
    client.check_msg()
    time.sleep(1)


from umqtt.simple import MQTTClient

# Test reception e.g. with:
# mosquitto_sub -t foo_topic


def main(server="192.168.43.31"):
    c = MQTTClient("umqtt_client", server)
    c.connect()
    c.publish(b"foo_topic", b"hello")
    c.disconnect()


if __name__ == "__main__":
    main()
