## Open-SDI12-Blue with Softdevice S113 V7.2.0

The nRF52832 can be run with Softdevice S112 and Softdevice S113,
whereas nRF52840 only can be used with S113.
S113 V7.2.0 simplys needs 340 Bytes more RAM and 12.288 Bytes more Flash.

This Project (with S113) contains only "DEVICE_TYP 200". Others can be added as in the S112 Project.


Differences in Project File: 

S112:
- c_preprocessor_definitions=" ... ;NRF_SD_BLE_API_VERSION=7;S112;SOFTDEVICE_PRESENT; ..."
- linker_section_placement_macros="FLASH_PH_START=0x0;FLASH_PH_SIZE=0x80000;RAM_PH_START=0x20000000;RAM_PH_SIZE=0x10000;FLASH_START=0x19000;FLASH_SIZE=0x67000;RAM_START=0x200022c8;RAM_SIZE=0xdd38"
- debug_additional_load_file="../../../../../../components/softdevice/s112/hex/s112_nrf52_7.2.0_softdevice.hex"

S113:
- c_preprocessor_definitions=" ... ;NRF_SD_BLE_API_VERSION=7;S113;SOFTDEVICE_PRESENT; ..."
- linker_section_placement_macros="FLASH_PH_START=0x0;FLASH_PH_SIZE=0x80000;RAM_PH_START=0x20000000;RAM_PH_SIZE=0x10000;FLASH_START=0x1C000;FLASH_SIZE=0x64000;RAM_START=0x20002608;RAM_SIZE=0xd9f8"
- debug_additional_load_file="../../../../../../components/softdevice/s113/hex/s113_nrf52_7.2.0_softdevice.hex"
      
***
