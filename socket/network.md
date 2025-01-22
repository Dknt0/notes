Network Configuration Note
===

> TODO: Learn more about network configuration.
> Dknt 2025.01.11

Commonly used commands:

- `ping`: test network connectivity
- `ip`: operate net interfaces
- `iptables` / `ip6tables`: configure IP packet filter rules
- `ufw`: manage firewall
- `route`: configure routing table
- `netstat`: show network statistics
- `ss`: show network connections
- `nmap`: network scanner
- `nslookup`: query DNS information
- `dig`: query DNS information
- `tcpdump`: capture network traffic
- `wireshark`: network protocol analyzer
- `netcat`: network tool
- `curl`: transfer data with URLs
- `wget`: download files from the web
- `iperf`: network performance measurement
- `socat`: network tool
- `ssh`: secure shell
- `scp`: secure copy
- `rsync`: remote sync
- `sshfs`: secure file system
- `sshpass`: password authentication
- `expect`: automate interaction with programs

Use command `ip` to operate net interfaces:

```bash
# Show all interfaces
ip link
# Show interface details
ip -s link
```

Use command `ip` to configure a router:

```bash
# Add a route
ip route add 192.168.1.0/24 via 192.168.1.1
# Delete a route
ip route del 192.168.1.0/24 via 192.168.1.1
# Flush the routing table
ip route flush
# Show the routing table
ip route
# Show the routing table with statistics
ip -s route
```

`iptables` and `ip6tables` are used to configure the IP packet filter rules. The difference between `ip` and `iptables` is that `ip` is used to configure the net interfaces, while `iptables` is used to configure the IP packet filter rules. Usage:

```bash
# Show the iptables rules
iptables -L
# Add a rule
iptables -A INPUT -s 192.168.1.0/24 -j ACCEPT
# Delete a rule
iptables -D INPUT -s 192.168.1.0/24 -j ACCEPT
# Flush the iptables rules
iptables -F
```

`ufw` is a firewall management tool for Ubuntu. Usage:

```bash
# Enable ufw
ufw enable
# Disable ufw
ufw disable
# Show the ufw rules
ufw status
# Add a rule
ufw allow from 192.168.1.0/24 to any port 22
# Delete a rule
ufw delete allow from 192.168.1.0/24 to any port 22
# Flush the ufw rules
ufw reset
```



