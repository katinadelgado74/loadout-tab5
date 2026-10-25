# Custom bootloader override for LOADOUT (based on ESP-IDF's
# examples/custom_bootloader/bootloader_override). Replaces the default
# bootloader "main" component so we can choose which app partition to boot.
idf_component_register(SRCS "bootloader_start.c"
                       REQUIRES bootloader bootloader_support spi_flash)

# Reuse the stock bootloader linker scripts for this target.
idf_build_get_property(target IDF_TARGET)
set(target_folder "${target}")
set(SCRIPTS
    "${IDF_PATH}/components/bootloader/subproject/main/ld/${target_folder}/bootloader.ld"
    "${IDF_PATH}/components/bootloader/subproject/main/ld/${target_folder}/bootloader.rom.ld")
target_linker_script(${COMPONENT_LIB} INTERFACE "${SCRIPTS}")
