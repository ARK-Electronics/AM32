/*
  sitl_config.c - JSON config file and command line handling for AM32 SITL
 */

#include "sitl_config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>

#include "jsmn.h"

sitl_config_t sitl_cfg = {
    .motor = {
        .kv = 900,
        .poles = 14,
        .resistance = 0.045f,
        .inductance = 2.1e-5f,
        .mutual_inductance = 0.0f,
        // defaults model a ~7 inch prop on a 900Kv motor at 4S: about
        // 19A and 13500 rpm at full throttle. Rotors much lighter than
        // this accelerate faster than the firmware desync detection
        // allows
        .inertia = 3.0e-5f,
        .damping = 1.0e-6f,
        .static_friction = 0.003f,
        .load_k_omega2 = 1.0e-7f,
    },
    .battery = {
        .voltage = 16.8f,
        .resistance = 0.012f,
    },
    .esc = {
        .rds_on = 0.004f,
        .diode_vf = 0.7f,
        .temperature_c = 25.0f,
    },
    .sim = {
        .physics_dt_ns = 500,
        .loop_time_ns = 2000,
        .isr_read_ns = 100,
        .comparator_noise_mv = 5.0f,
        // enough hysteresis that noise cannot eat a zero crossing edge,
        // as on a real comparator
        .comparator_hysteresis_mv = 15.0f,
        .watchdog_enabled = true,
    },
    .speedup = 1.0f,
    .input_port = 57733,
    .eeprom_path = "am32_eeprom.bin",
    .can_uri = "mcast:0",
    .uid = NULL,
    .node_id = -1,
    .input_type = -1,
    .verbose = false,
    .nosleep = false,
    .realtime = false,
};

struct cfg_entry {
    const char* section;
    const char* key;
    enum { CFG_FLOAT,
        CFG_INT,
        CFG_U32,
        CFG_BOOL } type;
    void* ptr;
};

static const struct cfg_entry cfg_table[] = {
    { "motor", "kv", CFG_FLOAT, &sitl_cfg.motor.kv },
    { "motor", "poles", CFG_INT, &sitl_cfg.motor.poles },
    { "motor", "resistance", CFG_FLOAT, &sitl_cfg.motor.resistance },
    { "motor", "inductance", CFG_FLOAT, &sitl_cfg.motor.inductance },
    { "motor", "mutual_inductance", CFG_FLOAT, &sitl_cfg.motor.mutual_inductance },
    { "motor", "inertia", CFG_FLOAT, &sitl_cfg.motor.inertia },
    { "motor", "damping", CFG_FLOAT, &sitl_cfg.motor.damping },
    { "motor", "static_friction", CFG_FLOAT, &sitl_cfg.motor.static_friction },
    { "motor", "load_k_omega2", CFG_FLOAT, &sitl_cfg.motor.load_k_omega2 },
    { "battery", "voltage", CFG_FLOAT, &sitl_cfg.battery.voltage },
    { "battery", "resistance", CFG_FLOAT, &sitl_cfg.battery.resistance },
    { "esc", "rds_on", CFG_FLOAT, &sitl_cfg.esc.rds_on },
    { "esc", "diode_vf", CFG_FLOAT, &sitl_cfg.esc.diode_vf },
    { "esc", "temperature_c", CFG_FLOAT, &sitl_cfg.esc.temperature_c },
    { "sim", "physics_dt_ns", CFG_U32, &sitl_cfg.sim.physics_dt_ns },
    { "sim", "loop_time_ns", CFG_U32, &sitl_cfg.sim.loop_time_ns },
    { "sim", "isr_read_ns", CFG_U32, &sitl_cfg.sim.isr_read_ns },
    { "sim", "comparator_noise_mv", CFG_FLOAT, &sitl_cfg.sim.comparator_noise_mv },
    { "sim", "comparator_hysteresis_mv", CFG_FLOAT, &sitl_cfg.sim.comparator_hysteresis_mv },
    { "sim", "watchdog_enabled", CFG_BOOL, &sitl_cfg.sim.watchdog_enabled },
};

static void set_value(const char* section, const char* js, const jsmntok_t* key, const jsmntok_t* val, const char* path)
{
    char keystr[64], valstr[64];
    snprintf(keystr, sizeof(keystr), "%.*s", key->end - key->start, js + key->start);
    snprintf(valstr, sizeof(valstr), "%.*s", val->end - val->start, js + val->start);
    for (unsigned i = 0; i < sizeof(cfg_table) / sizeof(cfg_table[0]); i++) {
        const struct cfg_entry* e = &cfg_table[i];
        if (strcmp(e->section, section) != 0 || strcmp(e->key, keystr) != 0) {
            continue;
        }
        switch (e->type) {
        case CFG_FLOAT:
            *(float*)e->ptr = strtof(valstr, NULL);
            break;
        case CFG_INT:
            *(int*)e->ptr = atoi(valstr);
            break;
        case CFG_U32:
            *(uint32_t*)e->ptr = strtoul(valstr, NULL, 0);
            break;
        case CFG_BOOL:
            *(bool*)e->ptr = (strcmp(valstr, "true") == 0 || strcmp(valstr, "1") == 0);
            break;
        }
        return;
    }
    fprintf(stderr, "SITL: %s: unknown config key %s.%s\n", path, section, keystr);
    exit(1);
}

// count the tokens making up one JSON value, for skipping
static int value_size(const jsmntok_t* t)
{
    int count = 1;
    if (t->type == JSMN_OBJECT) {
        const jsmntok_t* p = t + 1;
        for (int i = 0; i < t->size; i++) {
            count++; // key
            const int vs = value_size(p + 1);
            count += vs;
            p += 1 + vs;
        }
    } else if (t->type == JSMN_ARRAY) {
        const jsmntok_t* p = t + 1;
        for (int i = 0; i < t->size; i++) {
            const int vs = value_size(p);
            count += vs;
            p += vs;
        }
    }
    return count;
}

static void load_json(const char* path)
{
    FILE* f = fopen(path, "r");
    if (!f) {
        fprintf(stderr, "SITL: failed to open config %s\n", path);
        exit(1);
    }
    static char js[16384];
    const size_t n = fread(js, 1, sizeof(js) - 1, f);
    fclose(f);
    js[n] = 0;

    jsmn_parser parser;
    static jsmntok_t tokens[512];
    jsmn_init(&parser);
    const int ntok = jsmn_parse(&parser, js, n, tokens, 512);
    if (ntok < 1 || tokens[0].type != JSMN_OBJECT) {
        fprintf(stderr, "SITL: invalid JSON in %s (err %d)\n", path, ntok);
        exit(1);
    }

    const jsmntok_t* t = &tokens[1];
    for (int i = 0; i < tokens[0].size; i++) {
        const jsmntok_t* section = t;
        const jsmntok_t* sec_obj = t + 1;
        char secstr[64];
        snprintf(secstr, sizeof(secstr), "%.*s", section->end - section->start, js + section->start);
        if (sec_obj->type != JSMN_OBJECT) {
            fprintf(stderr, "SITL: %s: section %s is not an object\n", path, secstr);
            exit(1);
        }
        const jsmntok_t* kt = sec_obj + 1;
        for (int k = 0; k < sec_obj->size; k++) {
            const jsmntok_t* val = kt + 1;
            set_value(secstr, js, kt, val, path);
            kt += 1 + value_size(val);
        }
        t += 1 + value_size(sec_obj);
    }
}

static void usage(const char* prog)
{
    printf("Usage: %s [options]\n"
           "  --config FILE    JSON config file for motor/battery/esc/sim\n"
           "  --eeprom FILE    eeprom backing file (default am32_eeprom.bin)\n"
           "  --can-uri URI    CAN interface (default mcast:0)\n"
           "  --input-port N   UDP port for PWM/DShot input, 0 to disable\n"
           "                   (default 57733)\n"
           "  --speedup X      simulation speed, 0 for free running (default 1.0)\n"
           "  --node-id N      force DroneCAN node ID\n"
           "  --input-type N   force eeprom INPUT_SIGNAL_TYPE (0=auto 1=dshot\n"
           "                   2=servo 5=dronecan)\n"
           "  --uid STR        string used to derive the 16 byte unique ID\n"
           "  --verbose        1Hz state output on stderr\n"
           "  --nosleep        busy wait instead of sleeping (uses two full\n"
           "                   CPU cores but gives the most accurate timing)\n"
           "  --realtime       SCHED_FIFO scheduling for both threads (needs\n"
           "                   root or an rtprio rlimit; with --nosleep also\n"
           "                   set kernel.sched_rt_runtime_us=-1)\n",
        prog);
}

void sitl_config_init(int argc, char** argv)
{
    static const struct option opts[] = {
        { "config", required_argument, NULL, 'c' },
        { "eeprom", required_argument, NULL, 'e' },
        { "can-uri", required_argument, NULL, 'u' },
        { "input-port", required_argument, NULL, 'p' },
        { "speedup", required_argument, NULL, 's' },
        { "node-id", required_argument, NULL, 'n' },
        { "input-type", required_argument, NULL, 'I' },
        { "uid", required_argument, NULL, 'U' },
        { "verbose", no_argument, NULL, 'v' },
        { "nosleep", no_argument, NULL, 'N' },
        { "realtime", no_argument, NULL, 'R' },
        { "help", no_argument, NULL, 'h' },
        { NULL, 0, NULL, 0 },
    };
    int c;
    while ((c = getopt_long(argc, argv, "c:e:u:p:s:n:I:U:vNRh", opts, NULL)) != -1) {
        switch (c) {
        case 'c':
            load_json(optarg);
            break;
        case 'e':
            sitl_cfg.eeprom_path = optarg;
            break;
        case 'u':
            sitl_cfg.can_uri = optarg;
            break;
        case 'p':
            sitl_cfg.input_port = atoi(optarg);
            break;
        case 's':
            sitl_cfg.speedup = strtof(optarg, NULL);
            break;
        case 'n':
            sitl_cfg.node_id = atoi(optarg);
            break;
        case 'I':
            sitl_cfg.input_type = atoi(optarg);
            break;
        case 'U':
            sitl_cfg.uid = optarg;
            break;
        case 'v':
            sitl_cfg.verbose = true;
            break;
        case 'N':
            sitl_cfg.nosleep = true;
            break;
        case 'R':
            sitl_cfg.realtime = true;
            break;
        case 'h':
        default:
            usage(argv[0]);
            exit(c == 'h' ? 0 : 1);
        }
    }
}
