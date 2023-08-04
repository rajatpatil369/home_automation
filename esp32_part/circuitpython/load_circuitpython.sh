# https://learn.adafruit.com/circuitpython-with-esp32-quick-start/command-line-esptool

port=$1
bin=$2

echo port=$port
echo firmware=$bin

esptool.py --chip esp32 --port $port erase_flash
esptool.py --port $port write_flash -z 0x0 $bin
