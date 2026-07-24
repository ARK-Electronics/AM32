/*
 * Firmware version for releases.
 *
 * MAJOR.MINOR track the upstream AM32 base this tree is based on (EEPROM and
 * protocol still use only these two numeric fields).
 * VERSION_TAG is appended to built artifact names (e.g. 3.0-ark) so binaries
 * from the ARK Electronics fork are not confused with upstream AM32 builds.
 * Update this file for new releases.
 */
#define VERSION_MAJOR 3
#define VERSION_MINOR 0
#define VERSION_TAG "ark"

#define EEPROM_VERSION 3
