import network

WIFI_SSID = 'Nokia 1'
WIFI_PASSWORD = '10:41 M 26-dec-22'

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
if not wlan.isconnected():
    print('connecting to network...')
    wlan.connect(WIFI_SSID, WIFI_PASSWORD)
    while not wlan.isconnected():
        pass
print('network config:', wlan.ifconfig())    # (IP address, subnet mask, gateway, DNS server)
