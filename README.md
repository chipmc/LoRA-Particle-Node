# LoRA-Particle-Node

A Particle project named LoRA-Particle-Node

This code is for a simple low-power LoRA node that communciates with the Gateway.  It will publish updates on a schedule set by the gateway and the gateway will, in turn, connect to Particle to send the data via-webhook

In this system, every communication is initiated by the node.  The Gateway will receive the transmission and reply with an acknowledgement.  This message will set the nodes' clock, send any configuration updates and let the node know to start the next reporting period.

## Release Notes

### v18

- Added node resiliency hardening for LoRa retry, direct-to-gateway post-ACK sleep, and closed-hours scheduling behavior.
- Preserved node identity and last known schedule across firmware updates by repairing invalid persisted frequency values in place instead of reinitializing sysStatus.
- Added reusable node PowerManager support for normalized battery telemetry, conservative fallback SOC handling, and Boron PMIC input profile selection for UsbBench, Solar35W, and Auto modes.
- Reduced PMIC log noise by caching the last applied profile and only reapplying SystemPowerConfiguration on cold boot or profile changes.

