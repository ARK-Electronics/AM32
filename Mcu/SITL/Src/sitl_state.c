/*
  sitl_state.c - high rate simulation state streaming and runtime model
  control over UDP, for GUI graphs and animations

  A client subscribes by sending a SUBSCRIBE packet with the desired
  sample period in simulated nanoseconds; the simulation thread then
  streams batched state samples (rotor angle/speed, phase currents, bus
  voltage/current, bridge modes, comparator) to the subscriber. The
  subscription expires two seconds after the last refresh.

  A LOAD_MODEL packet carries the path of a motor/battery/esc JSON file
  (same format as --config, sim section ignored) which is applied to the
  running simulation.

  client -> SITL (little endian):
    u16 magic 0x5353, u8 cmd, u8 pad, payload
      cmd 0 SUBSCRIBE: u32 period_ns; flags byte bit0 = averaged
        sampling (currents/voltages are the mean over each sample
        period instead of instantaneous, avoiding PWM aliasing at
        coarse periods)
      cmd 1 LOAD_MODEL: JSON file path (rest of packet)
      cmd 2 SET_SPEEDUP: float speedup (0 = free run)
  SITL -> client:
    u16 magic 0x5354, u8 version=1, u8 count, count * sample
    u16 magic 0x5355, u8 ok, u8 pad, message   (LOAD_MODEL reply)
*/

#include "sitl.h"
#include "sitl_config.h"
#include "motor.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define STATE_MAGIC_CMD 0x5353
#define STATE_MAGIC_DATA 0x5354
#define STATE_MAGIC_REPLY 0x5355

struct __attribute__((packed)) state_sample {
	uint64_t t_ns;
	float omega;	  // mechanical rad/s
	float theta;	  // mechanical angle, rad [0,2pi)
	float theta_e;	  // electrical angle, rad [0,2pi)
	float iu, iv, iw; // phase currents, A
	float vu, vv, vw; // phase terminal voltages, V
	float vbus, ibus;
	uint8_t modes[3];   // sitl_phase_mode per phase
	uint8_t comp_phase; // floating phase
	uint8_t comp_out;
	uint8_t pad[3];
};

#define STATE_BATCH 16

static int fd = -1;
static struct sockaddr_in sub_addr;
static bool have_sub;
static time_t sub_expire;
static uint32_t period_req_ns = 50000; // requested by the subscriber
static uint32_t period_ns = 50000;     // effective, wall rate limited
static bool averaged;		       // mean over the period instead of point samples
static double sig_acc[8];
static uint32_t sig_n;
static uint64_t next_sample_ns;
static uint64_t last_flush_ns;

static struct __attribute__((packed)) {
	uint16_t magic;
	uint8_t version;
	uint8_t count;
	struct state_sample s[STATE_BATCH];
} batch = {.magic = STATE_MAGIC_DATA, .version = 2};

void sitl_state_init(void)
{
	if (sitl_cfg.state_port <= 0) {
		return;
	}
	fd = sitl_udp_socket();
	if (fd < 0) {
		perror("SITL: state socket");
		return;
	}
	// no SO_REUSEADDR: a second instance on the same port must fail
	// loudly instead of silently stealing datagrams
	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons((uint16_t)sitl_cfg.state_port);
	addr.sin_addr.s_addr = htonl(sitl_cfg.bind_any ? INADDR_ANY : INADDR_LOOPBACK);
	if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
		perror("SITL: state bind");
		close(fd);
		fd = -1;
		return;
	}
	fprintf(stderr, "SITL: state/model port udp %d\n", sitl_cfg.state_port);
}

static void load_model(const char *path, struct sockaddr_in *src)
{
	char msg[256];
	const bool ok = sitl_config_reload(path);
	if (ok) {
		motor_config_changed();
		snprintf(msg, sizeof(msg), "loaded %.200s", path);
		fprintf(stderr, "SITL: %s\n", msg);
	} else {
		snprintf(msg, sizeof(msg), "failed to load %.200s", path);
		fprintf(stderr, "SITL: %s\n", msg);
	}
	struct __attribute__((packed)) {
		uint16_t magic;
		uint8_t ok;
		uint8_t pad;
		char msg[256];
	} reply = {.magic = STATE_MAGIC_REPLY, .ok = ok, .pad = 0};
	strncpy(reply.msg, msg, sizeof(reply.msg) - 1);
	sendto(fd, &reply, 4 + strlen(reply.msg) + 1, 0, (struct sockaddr *)src, sizeof(*src));
}

/*
  effective sample period: the subscriber's request, floored to the
  physics step and rate limited to about 200k samples per second of
  wall clock so fine sampling does not overload the sim thread at high
  speedups. Slowing the simulation down automatically allows finer
  sampling
 */
static void apply_period(void)
{
	uint32_t period = period_req_ns;
	if (period < sitl_cfg.sim.physics_dt_ns) {
		period = sitl_cfg.sim.physics_dt_ns;
	}
	const uint32_t wall_floor = (uint32_t)(5000.0f * sitl_cfg.speedup);
	if (sitl_cfg.speedup > 0 && period < wall_floor) {
		period = wall_floor;
	}
	period_ns = period;
}

// called from the sim thread every 100us
void sitl_state_poll(void)
{
	if (fd < 0) {
		return;
	}
	if (have_sub && time(NULL) > sub_expire) {
		have_sub = false;
	}
	uint8_t pkt[512];
	struct sockaddr_in src;
	socklen_t srclen = sizeof(src);
	const ssize_t ret = recvfrom(fd, pkt, sizeof(pkt) - 1, MSG_DONTWAIT, (struct sockaddr *)&src, &srclen);
	if (ret < 4) {
		return;
	}
	uint16_t magic;
	memcpy(&magic, pkt, 2);
	if (magic != STATE_MAGIC_CMD) {
		return;
	}
	const uint8_t cmd = pkt[2];
	if (cmd == 0 && ret >= 8) {
		memcpy(&period_req_ns, pkt + 4, 4);
		averaged = (pkt[3] & 1) != 0;
		apply_period();
		// a new subscriber must not receive samples batched for the
		// previous one
		if (!have_sub || src.sin_addr.s_addr != sub_addr.sin_addr.s_addr || src.sin_port != sub_addr.sin_port) {
			batch.count = 0;
			memset(sig_acc, 0, sizeof(sig_acc));
			sig_n = 0;
			next_sample_ns = 0;
		}
		sub_addr = src;
		have_sub = true;
		sub_expire = time(NULL) + 2;
	} else if (cmd == 1) {
		pkt[ret] = 0;
		load_model((const char *)(pkt + 4), &src);
	} else if (cmd == 2 && ret >= 8) {
		float speedup;
		memcpy(&speedup, pkt + 4, 4);
		if (speedup >= 0 && speedup <= 100) {
			// the pacing loop rebases its references on change
			sitl_cfg.speedup = speedup;
			apply_period();
			fprintf(stderr, "SITL: speedup %.3f\n", (double)speedup);
		}
	}
}

// called from the sim thread on every physics step
void sitl_state_step(uint64_t now_ns)
{
	if (!have_sub) {
		return;
	}
	if (averaged) {
		motor_add_signals(sig_acc);
		sig_n++;
	}
	if (now_ns < next_sample_ns) {
		return;
	}
	next_sample_ns = now_ns + period_ns;

	struct state_sample *s = &batch.s[batch.count];
	memset(s, 0, sizeof(*s));
	s->t_ns = now_ns;
	float omega, theta, theta_e, i[3], v[3], vbus, ibus;
	motor_get_live_state(&omega, &theta, &theta_e, i, v, &vbus, &ibus);
	s->omega = omega;
	s->theta = theta;
	s->theta_e = theta_e;
	if (averaged && sig_n > 0) {
		s->iu = (float)(sig_acc[0] / sig_n);
		s->iv = (float)(sig_acc[1] / sig_n);
		s->iw = (float)(sig_acc[2] / sig_n);
		s->vu = (float)(sig_acc[3] / sig_n);
		s->vv = (float)(sig_acc[4] / sig_n);
		s->vw = (float)(sig_acc[5] / sig_n);
		s->vbus = (float)(sig_acc[6] / sig_n);
		s->ibus = (float)(sig_acc[7] / sig_n);
		memset(sig_acc, 0, sizeof(sig_acc));
		sig_n = 0;
	} else {
		s->iu = i[0];
		s->iv = i[1];
		s->iw = i[2];
		s->vu = v[0];
		s->vv = v[1];
		s->vw = v[2];
		s->vbus = vbus;
		s->ibus = ibus;
	}
	for (int p = 0; p < 3; p++) {
		s->modes[p] = sitl_phase_mode[p];
	}
	s->comp_phase = sitl_comp_phase;
	s->comp_out = sitl_comp_out;
	batch.count++;

	if (batch.count >= STATE_BATCH || now_ns - last_flush_ns > 5000000ULL) {
		sendto(fd, &batch, 4 + batch.count * sizeof(struct state_sample), 0, (struct sockaddr *)&sub_addr, sizeof(sub_addr));
		batch.count = 0;
		last_flush_ns = now_ns;
	}
}
