---
layout: post
cover: assets/images/ha.png
title: Setting up Home Assistant at your house
date: 2020-05-20 12:00:00 +0545
categories: hassio home automation raspberry
author: olaya
featured: true
summary: Automating you home with a low budget and using open-source tools

---
# Network configuration
## Static IP
From the [installation](https://www.home-assistant.io/hassio/installation/)
instructions we can see how to set up a static IP, so our router doesn't
reassing it every time we power OFF and ON the raspberry pi. The steps are as
follows:
1. Format an USB-Stick as a FAT32 partition called `CONFIG`.
2. create a file called ´my-network´ inside of a directory called `network`.


An example of ´my-network´ file for a WiFi connection with a static IP:

``ini
[connection]
id=my-network
uuid=72111c67-4a5d-4d5c-925e-f8ee26efb3c3
type=802-11-wireless

[802-11-wireless]
mode=infrastructure
ssid=MY_SSID
# Uncomment below if your SSID is not broadcasted
#hidden=true

[802-11-wireless-security]
auth-alg=open
key-mgmt=wpa-psk
psk=MY_WLAN_SECRET_KEY

[ipv4]
method=manual
address=192.168.8.102/24;192.168.8.1
dns=8.8.8.8;8.8.4.4;

[ipv6]
addr-gen-mode=stable-privacy
method=auto
```
The static IP is defined in the `ipv4` field.
For `address`, the value before the semicolon is the IP address and subnet prefix bitlength. The second value (after the semicolon) is the IP address of the local gateway.

## Remote access

Under construction.
