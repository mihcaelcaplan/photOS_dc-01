# hello-world_dc-01

This is a minimal working example for the DC-01 firmware, based on the IMXRT1060 SOC.

### Background

The DC-0x series of minimal digital cameras are lightweight, easy-to-use digital cameras for creative photographers. Each camera is simple, accessible, and convenient for both on-the-run (street) and static, composition-focused image creation. The image “tone” is constrained by and, consequently, enables the low-cost design, which unlocks new possibilities for dynamic, unique images. The DC-0x series allows the user to create memories, rather than just smartphone photos.

Detailed Software Requirements can be found in [Software Requirements](https://forested-lupin-e73.notion.site/Software-Requirements-1f3fa9aeb37480798af6cb0c6953db27).

## Getting Started

This project is based on **MCUXpresso Version 24.12.** 

The SOC configuration data used by the internal tools is in the `.mex` file. See [Configuration Tools](https://www.notion.so/Programming-Guide-19efa9aeb374801a982dc767bf2b354d?pvs=21).

To make the image bootable, vs. debuggable, you must change the Active Build Configuration as shown in [Setting Build Configuration ](https://www.notion.so/Programming-Guide-19efa9aeb374801a982dc767bf2b354d?pvs=21).

## Extending This Example

- Replace the hello world logic in `/source/hello_world_dc-01.c : main()` with your application code.
- Add your own modules to implement advanced functionality.
- Add SDK components as in [SDK Components](https://www.notion.so/Programming-Guide-19efa9aeb374801a982dc767bf2b354d?pvs=21).
- Can change memory regions, linker settings and execution from memory [Image Structure and Memory](https://www.notion.so/Image-Structure-and-Memory-1edfa9aeb37480cd91cdcb594340707d?.pvs=21)
- For all the details, see the full processor reference manual in `/documentation/IMXRT1060XRM.pdf`.