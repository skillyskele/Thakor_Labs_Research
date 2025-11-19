# SoC - Empty

This project starts with Simplicity Studio's Bluetooth SoC-Empty example. From there, the pin tool was used to download the software package for IADC, PRS, and LETIMER. It uses GNU ARM v12.2.1.
> Note: the SDK is v2025.6.2

The Bluetooth SoC-Empty example is a project that you can use as a template for any standalone Bluetooth application.

> Note: this example expects a specific Gecko Bootloader to be present on your device. For details see the Troubleshooting section.

## Documentation 
- More documentation is coming soon
- As of now, this is the current operation. It's very one-after-another, and Bluetooth doesn't seem to operate asynchronously from the other processes yet (may change in the future when sampling rates go higher).
* Timer triggers a cycle
* IADC-LDMA chain fills uint32_t buffer A with 60 samples (240 bytes)
* LDMA interrupt fires, switches to buffer B, sets flag blueToothNotif = True
* Bluetooth process gives packet an ID, sends packet, switches to the next buffer (buffer A since there's only 2 buffers for now), sets blueToothNotif = False
* Cycle Complete. Timer will trigger again


