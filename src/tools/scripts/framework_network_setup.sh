sudo ip addr add 192.168.1.2/24 dev enx9cbf0d007947
sudo ip route del 10.42.0.0/24 dev enx9cbf0d007947
sudo ip route del 169.254.0.0/16 dev enx9cbf0d007947
sudo ip route add 192.168.1.0/24 dev enx9cbf0d007947 src 192.168.1.2
sudo ip addr del 10.42.0.1/24 dev enx9cbf0d007947
ip route
