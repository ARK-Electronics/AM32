/*
  eeprom.c - SITL flash emulation backed by a file. The firmware reads and
  writes 192 bytes at eeprom_address; the offset within the backing file is
  relative to EEPROM_START_ADD
 */

#include "eeprom.h"

#include "sitl_config.h"
#include "targets.h"

#include <stdio.h>
#include <string.h>

void save_flash_nolib(uint8_t* data, int length, uint32_t add)
{
    const uint32_t offset = add - EEPROM_START_ADD;
    FILE* f = fopen(sitl_cfg.eeprom_path, "r+b");
    if (!f) {
        f = fopen(sitl_cfg.eeprom_path, "w+b");
    }
    if (!f) {
        perror("SITL: eeprom open");
        return;
    }
    fseek(f, offset, SEEK_SET);
    fwrite(data, 1, length, f);
    fclose(f);
}

// provided by DroneCAN.c on CAN builds: the AM32 configurator default
// settings, used to seed a missing eeprom file so first boot behaves like
// a factory flashed ESC rather than erased flash
const uint8_t* DroneCAN_default_settings(unsigned* len) __attribute__((weak));

void read_flash_bin(uint8_t* data, uint32_t add, int out_buff_len)
{
    const uint32_t offset = add - EEPROM_START_ADD;
    // erased flash reads as 0xFF
    memset(data, 0xFF, out_buff_len);
    FILE* f = fopen(sitl_cfg.eeprom_path, "rb");
    if (!f) {
        if (DroneCAN_default_settings != NULL && offset == 0) {
            unsigned len = 0;
            const uint8_t* def = DroneCAN_default_settings(&len);
            if ((int)len > out_buff_len) {
                len = out_buff_len;
            }
            memcpy(data, def, len);
            if (out_buff_len > 27) {
                // make the seeded settings describe the simulated motor,
                // as a properly configured ESC would: a mismatched
                // MOTOR_KV makes low rpm power protection clamp the duty
                // at the wrong rpm. eeprom offsets 26/27 = motor_kv/poles,
                // kv is stored as (kv-20)/40
                data[26] = (uint8_t)((sitl_cfg.motor.kv - 20.0f) / 40.0f + 0.5f);
                data[27] = (uint8_t)sitl_cfg.motor.poles;
            }
        }
        return;
    }
    fseek(f, offset, SEEK_SET);
    if (fread(data, 1, out_buff_len, f) < (size_t)out_buff_len) {
        // short file, rest stays 0xFF
    }
    fclose(f);
}
