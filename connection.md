# Connecting

Connect to the Rock 4 C+ through visual interface. 
Username: rock 
Pwd: rock

# IP

Check if eth0 is wired in:
mii-tool interface name

1000baseT-FD (full duplex) a un 1 GB, link ok. Should be 1 GB 

ifconfig or ip link show

set ip on pc:
ifconfig enx803f5df173d1 192.168.155.2

set ip on companion:
ifconfig eth0 192.168.155.1


sudo route -v add -net 192.168.155.0/24  dev enx803f5df173d1

ssh rock@192.168.155.1



# Wifi

Companion: to see to which wifi is connected to OptiTrack.speed

use command 
nmcli

Recover passwords: 
sudo grep -r '^psk=' /etc/NetworkManager/system-connections/

The output is 
/etc/NetworkManager/system-connections/OptiTrack.nmconnection:psk=60A84A244BECD

## Passwords: 

OptiTrack 60A84A244BECD
OptiTrack.speed 60A84A244BECD



# Launch

Move through terminals using 

ctrl + alt + F1 to F12 for 12 terminals


# Ros2 over wifi

## Firewall

Check firewall
sudo ufw status

deactivate
sudo systemctl stop ufw

Remember to reactivate
sudo systemctl start ufw

## Domain id

Set the same domain ID for both 

export ROS_DOMAIN_ID=0

