# put this in /home/default/proxy
# and launch it in /etc/init.d/rcS

proxy_path="/home/default/proxy"
port_gps=4567
port_wp=4568
device="/dev/ttyUSB0"

#insmod $modules_path/cdc-acm.ko
stty -F $device 115200

# transmit arduino commands to drone
# $proxy_path/arduinoProxy < $device &

# publish gps data
nc -l -l -p $port_gps -e cat $device &

# listen for waypoints
nc -l -p $port_wp > $device &
