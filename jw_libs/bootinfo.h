// Bootinfo.h - Get Firmware-Info 

// Structure

// Copied from JesFsBootloader-Headers..  (components\libraries\bootloader... ) and modified
#if defined( NRF52832_XXAA )
    #define BOOTLOADER_SETTINGS_ADDRESS     (0x0007F000UL)
    #define BOOTLOADER_FLASH_START          (0x00070000)

    #define IBOOT_FLASH_START          (0x00048000)  // User Area, internal Bootloader
    #define IBOOT_FLASH_SIZE            (160*1024UL)

#elif defined( NRF52833_XXAA )
    #define BOOTLOADER_SETTINGS_ADDRESS     (0x0007F000UL)
    #define BOOTLOADER_FLASH_START          (0x00070000)

    #define IBOOT_FLASH_START          (0x00048000)  // User Area, internal Bootloader
    #define IBOOT_FLASH_SIZE            (160*1024UL)

#elif defined(NRF52840_XXAA)
    #define BOOTLOADER_SETTINGS_ADDRESS     (0x000FF000UL)
    #define BOOTLOADER_FLASH_START          (0x000F0000)

    #define IBOOT_FLASH_START          (0x00099000)  // User Area, internal Bootloader
    #define IBOOT_FLASH_SIZE            (348*1024UL)

#else
    #error No valid target set for BOOTLOADER_SETTINGS_ADDRESS.
#endif

#define   CPU_SECTOR_SIZE 4096    // 1 Sector: 4k for all nrF52


uint32_t get_firmware_bootloader_cookie(void);	// get Firmware_cookie
uint8_t* get_security_key_ptr(void);
uint32_t get_pin(void);

// End
