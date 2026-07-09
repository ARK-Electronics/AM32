/*
  sitl_main.c - entry point for the AM32 SITL build. Sets up the
  simulation and then runs the firmware's main() (renamed to am32_main by
  targets.h)
 */

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/file.h>

#include "sitl.h"
#include "sitl_config.h"
#include "motor.h"

#include "eeprom.h"
#include "targets.h"

// targets.h renames the firmware main() to am32_main; this file provides
// the real process main()
#undef main

extern int am32_main(void);
extern void save_flash_nolib(uint8_t* data, int length, uint32_t add);
extern void read_flash_bin(uint8_t* data, uint32_t add, int out_buff_len);

// force settings in the eeprom backing file from the command line
static void apply_eeprom_overrides(void)
{
    EEprom_t buf;
    read_flash_bin(buf.buffer, EEPROM_START_ADD, sizeof(buf.buffer));
    bool changed = false;
    if (sitl_cfg.node_id >= 0 && buf.can.can_node != (uint8_t)sitl_cfg.node_id) {
        buf.can.can_node = (uint8_t)sitl_cfg.node_id;
        changed = true;
    }
    if (sitl_cfg.input_type >= 0 && buf.input_type != (uint8_t)sitl_cfg.input_type) {
        buf.input_type = (uint8_t)sitl_cfg.input_type;
        changed = true;
    }
    if (changed) {
        save_flash_nolib(buf.buffer, sizeof(buf.buffer), EEPROM_START_ADD);
    }
}

/*
  hold an exclusive lock on the eeprom file for the life of the process. A
  second instance sharing the eeprom (and therefore node ID) would corrupt
  the DroneCAN traffic with interleaved transfers, which shows up as
  erratic telemetry. The fd is CLOEXEC so a reset (re-exec) drops and
  immediately re-acquires it
 */
static void lock_instance(void)
{
    // a separate lock file, so the eeprom file itself is only created by
    // the firmware writing it (a missing eeprom triggers default seeding)
    char lockpath[512];
    snprintf(lockpath, sizeof(lockpath), "%s.lock", sitl_cfg.eeprom_path);
    const int fd = open(lockpath, O_RDWR | O_CREAT | O_CLOEXEC, 0644);
    if (fd < 0) {
        perror("SITL: lock file open");
        exit(1);
    }
    if (flock(fd, LOCK_EX | LOCK_NB) != 0) {
        fprintf(stderr,
            "SITL: %s is in use by another SITL instance. Use --eeprom and "
            "--node-id to run multiple instances\n",
            sitl_cfg.eeprom_path);
        exit(1);
    }
    // fd deliberately left open to hold the lock
}

int main(int argc, char** argv)
{
    sitl_saved_argv = argv;
    sitl_config_init(argc, argv);
    lock_instance();
    motor_init();
    if (sitl_cfg.node_id >= 0 || sitl_cfg.input_type >= 0) {
        apply_eeprom_overrides();
    }

    fprintf(stderr, "AM32 SITL: eeprom=%s can=%s speedup=%.1f\n",
        sitl_cfg.eeprom_path, sitl_cfg.can_uri, (double)sitl_cfg.speedup);

    sitl_input_init();
    sitl_state_init();
    sitl_start_sim_thread();
    return am32_main();
}
