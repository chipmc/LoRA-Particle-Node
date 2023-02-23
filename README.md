# LoRA-Particle-Node

A Particle project named LoRA-Particle-Node

This code is for a simple low-power LoRA node that communciates with the Gateway.  It will publish updates on a schedule set by the gateway and the gateway will, in turn, connect to Particle to send the data via-webhook

In this system, every communication is initiated by the node.  The Gateway will receive the transmission and reply with an acknowledgement.  This message will set the nodes' clock, send any configuration updates and let the node know to start the next reporting period.

