---
layout: post
cover: assets/images/flysky/flysky.jpg
title: Using your RC controller as an USB joystick
date: 2020-07-21 12:00:00 +0545
categories: flysky fs-i6x phdstuff
author: olaya
featured: true
summary: Using the flysky fs-i6x as a joystick in Ubuntu

---
Hello all! First of all, I want to note the new `phdstuff` label that I'm introducing in this post. The point
is to show you which of these publications are contributing to my PhD, even if they're trivial, because there's much
work behind a PhD than just the "journal-worthy" work. I feel like this kind of work is normally invisible,
or even undervalued, even if it takes loads of time and it's essential for the whole system to work.
Anyway, let's go with the post!

This post comes from my need to have an USB joystick more precise than a xbox controller, but cheaper than the professional hall effect joysticks.
The ideal solution for this seemed to be the RC joysticks typically used with drones. However, we need to interface somehow the radio signal to the PC.
Some RC controllers (the one of this post included) have a simulator cable. However, I was looking for a wireless solution, which I find more convenient to operate a robot.
In this post, we will get the wireless radio signal from the controller, and interface it to an Ubuntu PC.

# Hardware Setup
What we first need to do, is to connect the radio receiver that will get the signal from the controller to our PC.
We will do so with a TTL to USB adapter.
I used the following hardware:

- FlySky FS-i6x remote control. I'm using this one because I already had it at home, but note that the original post used a fs-i6 remote.
- FlySky FS-iA6B receiver. [A bit overpriced in my opinion](https://www.amazon.es/Tamlltide-FS-iA6B-Transmisor-FS-GT2E-compatible/dp/B078WKR48Y/ref=sr_1_3?__mk_es_ES=%C3%85M%C3%85%C5%BD%C3%95%C3%91&dchild=1&keywords=fs+ia6b&qid=1595337062&sr=8-3), for around 16 euros.
- USB to TTL adapter. [I got this one from DSD TECH](https://www.amazon.es/DSD-TECH-convertidor-Compatible-Windows/dp/B072K3Z3TL/ref=sr_1_5?__mk_es_ES=%C3%85M%C3%85%C5%BD%C3%95%C3%91&dchild=1&keywords=dsd+tech&qid=1595336578&sr=8-5) for ~8 euros. It already included the wires to connect with the receiver.

Connect the radio receiver to the TTL adapter as follows:

![](https://raw.githubusercontent.com/olayasturias/olayasturias.github.io/master/assets/images/flysky/connections.jpg)

That is, CH5 to GND, CH6 to +5V, and finally the rightmost pin to the RX pin in the adapter. The rest of the pins don't need to be connected.

Now, bind the receiver to your remote. With the receiver powered off, connect the jumper (provided with the receiver) as seen in the picture below.

![](https://raw.githubusercontent.com/olayasturias/olayasturias.github.io/master/assets/images/flysky/bindreceiver.jpg)

Connect the adapter to your PC to power up the receiver, and you'll see that it starts blinking fast. Now take the controller, and power it up while
pressing the BIND key:

![](https://raw.githubusercontent.com/olayasturias/olayasturias.github.io/master/assets/images/flysky/bind.jpg)

Now the controller screen should show you that it has been correctly binded.

And that's it with the hardware setup!!

# Software setup

```ini
sudo modprobe serio
sudo inputattach --fsia6b /dev/ttyUSBx
```
This creates a joystick device, which you can check with the `jstest-gtk` command



![](https://raw.githubusercontent.com/olayasturias/olayasturias.github.io/master/assets/images/flysky/axis.jpg)
