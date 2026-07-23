/*
 *	wmediumd, wireless medium simulator for mac80211_hwsim kernel module
 *	Copyright (c) 2011 cozybit Inc.
 *
 *	Author:	Javier Lopez	<jlopex@cozybit.com>
 *		Javier Cardona	<javier@cozybit.com>
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version 2
 *	of the License, or (at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 *	02110-1301, USA.
 */

#include <netlink/netlink.h>
#include <netlink/genl/genl.h>
#include <netlink/genl/ctrl.h>
#include <netlink/genl/family.h>
#include <linux/nl80211.h>
#include <sys/utsname.h>
#include <stdint.h>
#include <getopt.h>
#include <signal.h>
#include <event.h>
#include <math.h>
#include <sys/timerfd.h>
#include <errno.h>
#include <limits.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>

#include "wmediumd.h"
#include "ieee80211.h"
#include "config.h"
#include "wserver.h"
#include "wmediumd_dynamic.h"
#include "wserver_messages.h"

struct sockaddr_in serverAddr, clientAddr;
socklen_t len;
void* out_buf;
char in_buf[PAGE_SIZE];
static bool is_ap = true;

static inline int div_round(int a, int b)
{
	return (a + b - 1) / b;
}

static inline int pkt_duration(struct wmediumd *ctx, int len, int rate)
{
	/* preamble + signal + t_sym * n_sym, rate in 100 kbps */
	return 16 + 4 + 4 * div_round((16 + 8 * len + 6) * 10, 4 * rate);
}

int w_logf(struct wmediumd *ctx, u8 level, const char *format, ...)
{
	va_list(args);
	va_start(args, format);
	if (ctx->log_lvl >= level) {
		return vprintf(format, args);
	}
	return -1;
}

int w_flogf(struct wmediumd *ctx, u8 level, FILE *stream, const char *format, ...)
{
	va_list(args);
	va_start(args, format);
	if (ctx->log_lvl >= level) {
		return vfprintf(stream, format, args);
	}
	return -1;
}

static void wqueue_init(struct wqueue *wqueue, int cw_min, int cw_max)
{
	INIT_LIST_HEAD(&wqueue->frames);
	wqueue->cw_min = cw_min;
	wqueue->cw_max = cw_max;
}

void station_init_queues(struct station *station)
{
	wqueue_init(&station->queues[IEEE80211_AC_BK], 15, 1023);
	wqueue_init(&station->queues[IEEE80211_AC_BE], 15, 1023);
	wqueue_init(&station->queues[IEEE80211_AC_VI], 7, 15);
	wqueue_init(&station->queues[IEEE80211_AC_VO], 3, 7);
}

bool timespec_before(struct timespec *t1, struct timespec *t2)
{
	return t1->tv_sec < t2->tv_sec ||
	       (t1->tv_sec == t2->tv_sec && t1->tv_nsec < t2->tv_nsec);
}

void timespec_add_usec(struct timespec *t, int usec)
{
	t->tv_nsec += usec * 1000;
	if (t->tv_nsec >= 1000000000) {
		t->tv_sec++;
		t->tv_nsec -= 1000000000;
	}
}

// a - b = c
static int timespec_sub(struct timespec *a, struct timespec *b,
			struct timespec *c)
{
	c->tv_sec = a->tv_sec - b->tv_sec;

	if (a->tv_nsec < b->tv_nsec) {
		c->tv_sec--;
		c->tv_nsec = 1000000000 + a->tv_nsec - b->tv_nsec;
	} else {
		c->tv_nsec = a->tv_nsec - b->tv_nsec;
	}

	return 0;
}

void rearm_timer(struct wmediumd *ctx)
{
	struct timespec min_expires;
	struct itimerspec expires;
	struct station *station;
	struct frame *frame;
	int i;

	bool set_min_expires = false;

	/*
	 * Iterate over all the interfaces to find the next frame that
	 * will be delivered, and set the timerfd accordingly.
	 */
	list_for_each_entry(station, &ctx->stations, list) {
		for (i = 0; i < IEEE80211_NUM_ACS; i++) {
			frame = list_first_entry_or_null(&station->queues[i].frames,
							 struct frame, list);

			if (frame && (!set_min_expires ||
				      timespec_before(&frame->expires,
						      &min_expires))) {
				set_min_expires = true;
				min_expires = frame->expires;
			}
		}
	}

	if (set_min_expires) {
		memset(&expires, 0, sizeof(expires));
		expires.it_value = min_expires;
		timerfd_settime(ctx->timerfd, TFD_TIMER_ABSTIME, &expires,
				NULL);
	}
}

static inline bool frame_has_a4(struct frame *frame)
{
	struct ieee80211_hdr *hdr = (void *)frame->data;

	return (hdr->frame_control[1] & (FCTL_TODS | FCTL_FROMDS)) ==
		(FCTL_TODS | FCTL_FROMDS);
}

static inline bool frame_is_mgmt(struct frame *frame)
{
	struct ieee80211_hdr *hdr = (void *)frame->data;

	return (hdr->frame_control[0] & FCTL_FTYPE) == FTYPE_MGMT;
}

static inline bool frame_is_data(struct frame *frame)
{
	struct ieee80211_hdr *hdr = (void *)frame->data;

	return (hdr->frame_control[0] & FCTL_FTYPE) == FTYPE_DATA;
}

static inline bool frame_is_data_qos(struct frame *frame)
{
	struct ieee80211_hdr *hdr = (void *)frame->data;

	return (hdr->frame_control[0] & (FCTL_FTYPE | STYPE_QOS_DATA)) ==
		(FTYPE_DATA | STYPE_QOS_DATA);
}

static inline u8 *frame_get_qos_ctl(struct frame *frame)
{
	struct ieee80211_hdr *hdr = (void *)frame->data;

	if (frame_has_a4(frame))
		return (u8 *)hdr + 30;
	else
		return (u8 *)hdr + 24;
}

static enum ieee80211_ac_number frame_select_queue_80211(struct frame *frame)
{
	u8 *p;
	int priority;

	if (!frame_is_data(frame))
		return IEEE80211_AC_VO;

	if (!frame_is_data_qos(frame))
		return IEEE80211_AC_BE;

	p = frame_get_qos_ctl(frame);
	priority = *p & QOS_CTL_TAG1D_MASK;

	return ieee802_1d_to_ac[priority];
}

static double dBm_to_milliwatt(int decibel_intf)
{
#define INTF_LIMIT (31)
	int intf_diff = NOISE_LEVEL - decibel_intf;

	if (intf_diff >= INTF_LIMIT)
		return 0.001;

	if (intf_diff <= -INTF_LIMIT)
		return 1000.0;

	return pow(10.0, -intf_diff / 10.0);
}

static double milliwatt_to_dBm(double value)
{
	return 10.0 * log10(value);
}

static int set_interference_duration(struct wmediumd *ctx, int src_idx,
				     int duration, int signal)
{
	int i, medium_id;

	if (!ctx->intf)
		return 0;

	if (signal >= CCA_THRESHOLD)
		return 0;

    medium_id = ctx->sta_array[src_idx]->medium_id;
	for (i = 0; i < ctx->num_stas; i++) {
        if (medium_id != ctx->sta_array[i]->medium_id)
            continue;
		ctx->intf[ctx->num_stas * src_idx + i].duration += duration;
		// use only latest value
		ctx->intf[ctx->num_stas * src_idx + i].signal = signal;
	}

	return 1;
}

static int get_signal_offset_by_interference(struct wmediumd *ctx, int src_idx,
					     int dst_idx)
{
    int i, medium_id;
	double intf_power;

	if (!ctx->intf)
		return 0;

	intf_power = 0.0;
    medium_id = ctx->sta_array[dst_idx]->medium_id;
	for (i = 0; i < ctx->num_stas; i++) {
		if (i == src_idx || i == dst_idx)
			continue;
        if (medium_id != ctx->sta_array[i]->medium_id)
            continue;
		if (drand48() < ctx->intf[i * ctx->num_stas + dst_idx].prob_col)
			intf_power += dBm_to_milliwatt(
				ctx->intf[i * ctx->num_stas + dst_idx].signal);
	}

	if (intf_power <= 1.0)
		return 0;

	return (int)(milliwatt_to_dBm(intf_power) + 0.5);
}

bool is_multicast_ether_addr(const u8 *addr)
{
	return 0x01 & addr[0];
}

static struct station *get_station_by_addr(struct wmediumd *ctx, u8 *addr)
{
	struct station *station;

	list_for_each_entry(station, &ctx->stations, list) {
		if (memcmp(station->addr, addr, ETH_ALEN) == 0)
			return station;
	}
	return NULL;
}

/*
 * hwsim forwards PMSR requests to the registered medium only on kernels
 * >= 6.4. Gate the handler on the running kernel so older hosts are
 * unaffected. The result is probed once and cached.
 */
static bool kernel_supports_pmsr(void)
{
	static int supported = -1;
	struct utsname uts;
	int major = 0, minor = 0;

	if (supported < 0) {
		supported = (uname(&uts) == 0 &&
			     sscanf(uts.release, "%d.%d", &major, &minor) == 2 &&
			     (major > 6 || (major == 6 && minor >= 4)));
	}
	return supported;
}

/* PMSR requests carry the initiator radio's permanent (hw) address. */
static struct station *get_station_by_hwaddr(struct wmediumd *ctx, u8 *hwaddr)
{
	struct station *station;

	list_for_each_entry(station, &ctx->stations, list) {
		if (memcmp(station->hwaddr, hwaddr, ETH_ALEN) == 0)
			return station;
	}
	return NULL;
}

static double sta_distance(struct station *a, struct station *b)
{
	double dx = a->x - b->x;
	double dy = a->y - b->y;
	double dz = a->z - b->z;

	return sqrt(dx * dx + dy * dy + dz * dz);
}

/*
 * Answer an FTM peer-measurement (PMSR) request from station geometry.
 *
 * The kernel sends HWSIM_CMD_START_PMSR with the initiator radio in
 * HWSIM_ATTR_ADDR_TRANSMITTER and an nl80211 PMSR request nested in
 * HWSIM_ATTR_PMSR_REQUEST. For each requested peer we compute the true
 * distance from the medium's station table, convert it to a round-trip
 * time, and return it in a HWSIM_CMD_REPORT_PMSR message carrying an
 * nl80211 FTM result. Secure ranging (secure LTF) is modelled by
 * reporting the geometric distance: an initiator cannot make a peer
 * appear closer than it truly is.
 *
 * The nested nl80211 TLV encoding follows the kernel PMSR layout; the
 * exact framing must be validated against a live >= 6.4 kernel with
 * PMSR-capable radios.
 */
static void hwsim_handle_pmsr_request(struct wmediumd *ctx,
				      struct nlmsghdr *nlh)
{
	struct nlattr *attrs[HWSIM_ATTR_MAX + 1];
	struct nlattr *pmsr_req, *peer, *result, *result_peers;
	struct station *req_sta;
	struct nl_msg *msg;
	u8 *transmitter;
	int rem, idx = 0;

	if (genlmsg_parse(nlh, 0, attrs, HWSIM_ATTR_MAX, NULL) < 0)
		return;
	if (!attrs[HWSIM_ATTR_ADDR_TRANSMITTER] ||
	    !attrs[HWSIM_ATTR_PMSR_REQUEST])
		return;

	transmitter = (u8 *)nla_data(attrs[HWSIM_ATTR_ADDR_TRANSMITTER]);
	req_sta = get_station_by_hwaddr(ctx, transmitter);
	if (!req_sta)
		return;

	/*
	 * HWSIM_ATTR_PMSR_REQUEST wraps an NL80211_ATTR_PEER_MEASUREMENTS
	 * container that holds one NL80211_PMSR_ATTR_PEERS attribute per peer.
	 */
	pmsr_req = nla_find(nla_data(attrs[HWSIM_ATTR_PMSR_REQUEST]),
			    nla_len(attrs[HWSIM_ATTR_PMSR_REQUEST]),
			    NL80211_ATTR_PEER_MEASUREMENTS);
	if (!pmsr_req)
		return;

	msg = nlmsg_alloc();
	if (!msg)
		return;

	if (genlmsg_put(msg, NL_AUTO_PID, NL_AUTO_SEQ, ctx->family_id, 0,
			NLM_F_REQUEST, HWSIM_CMD_REPORT_PMSR, VERSION_NR) == NULL)
		goto out;

	nla_put(msg, HWSIM_ATTR_ADDR_TRANSMITTER, ETH_ALEN, transmitter);

	result = nla_nest_start(msg, HWSIM_ATTR_PMSR_RESULT);
	result_peers = nla_nest_start(msg, NL80211_PMSR_ATTR_PEERS);

	nla_for_each_nested(peer, pmsr_req, rem) {
		struct nlattr *pa[NL80211_PMSR_PEER_ATTR_MAX + 1];
		struct nlattr *rpeer, *resp, *data, *ftm;
		struct station *peer_sta;
		u8 *peer_addr;
		double dist;
		int64_t rtt_ps;

		if (nla_type(peer) != NL80211_PMSR_ATTR_PEERS)
			continue;
		if (nla_parse_nested(pa, NL80211_PMSR_PEER_ATTR_MAX, peer,
				     NULL) < 0)
			continue;
		if (!pa[NL80211_PMSR_PEER_ATTR_ADDR])
			continue;

		peer_addr = (u8 *)nla_data(pa[NL80211_PMSR_PEER_ATTR_ADDR]);
		peer_sta = get_station_by_addr(ctx, peer_addr);
		if (!peer_sta)
			peer_sta = get_station_by_hwaddr(ctx, peer_addr);
		if (!peer_sta)
			continue;

		dist = sta_distance(req_sta, peer_sta);
		rtt_ps = (int64_t)(2.0 * dist / SPEED_OF_LIGHT_M_PER_S * 1e12);

		rpeer = nla_nest_start(msg, ++idx);
		nla_put(msg, NL80211_PMSR_PEER_ATTR_ADDR, ETH_ALEN, peer_addr);
		resp = nla_nest_start(msg, NL80211_PMSR_PEER_ATTR_RESP);
		nla_put_u32(msg, NL80211_PMSR_RESP_ATTR_STATUS,
			    NL80211_PMSR_STATUS_SUCCESS);
		data = nla_nest_start(msg, NL80211_PMSR_RESP_ATTR_DATA);
		ftm = nla_nest_start(msg, NL80211_PMSR_TYPE_FTM);
		nla_put_u32(msg, NL80211_PMSR_FTM_RESP_ATTR_NUM_FTMR_ATTEMPTS, 1);
		nla_put_u32(msg, NL80211_PMSR_FTM_RESP_ATTR_NUM_FTMR_SUCCESSES, 1);
		nla_put(msg, NL80211_PMSR_FTM_RESP_ATTR_RTT_AVG,
			sizeof(rtt_ps), &rtt_ps);
		nla_nest_end(msg, ftm);
		nla_nest_end(msg, data);
		nla_nest_end(msg, resp);
		nla_nest_end(msg, rpeer);
	}

	nla_nest_end(msg, result_peers);
	nla_nest_end(msg, result);

	nl_send_auto_complete(ctx->sock, msg);
out:
	nlmsg_free(msg);
}

void detect_mediums(struct wmediumd *ctx, struct station *src, struct station *dest) {
    int medium_id;
    if (!ctx->enable_medium_detection){
        return;
    }
    if(src->isap& !dest->isap){
        // AP-STA Connection
        medium_id = -src->index-1;
    }else if((!src->isap)& dest->isap){
        // STA-AP Connection
        medium_id = -dest->index-1;
    }else{
        // AP-AP Connection
        // STA-STA Connection
        // TODO: Detect adhoc and mesh groups
        return;
    }
    if (medium_id!=src->medium_id){
        w_logf(ctx, LOG_DEBUG, "Setting medium id of " MAC_FMT "(%d|%s) to %d.\n",
               MAC_ARGS(src->addr), src->index, src->isap ? "AP" : "Sta",
               medium_id);
        src-> medium_id = medium_id;
    }
    if(medium_id!=dest->medium_id){
        w_logf(ctx, LOG_DEBUG, "Setting medium id of " MAC_FMT "(%d|%s) to %d.\n",
               MAC_ARGS(dest->addr), dest->index, dest->isap ? "AP" : "Sta",
               medium_id);
        dest-> medium_id = medium_id;
    }
}
void queue_frame(struct wmediumd *ctx, struct station *station,
		 struct frame *frame)
{
	struct ieee80211_hdr *hdr = (void *)frame->data;
	u8 *dest = hdr->addr1;
	struct timespec now, target;
	struct wqueue *queue;
	struct frame *tail;
	struct station *tmpsta, *deststa;
	int send_time;
	int cw;
	double error_prob;
	bool is_acked = false;
	bool noack = false;
	int i, j;
	int rate_idx;
	int ac;

	/* TODO configure phy parameters */
	int slot_time = 9;
	int sifs = 16;
	int difs = 2 * slot_time + sifs;

	int retries = 0;

	clock_gettime(CLOCK_MONOTONIC, &now);

	int ack_time_usec = pkt_duration(ctx, 14, index_to_rate(0, frame->freq)) +
			sifs;

	/*
	 * To determine a frame's expiration time, we compute the
	 * number of retries we might have to make due to radio conditions
	 * or contention, and add backoff time accordingly.  To that, we
	 * add the expiration time of the previous frame in the queue.
	 */

	ac = frame_select_queue_80211(frame);
	queue = &station->queues[ac];

	/* try to "send" this frame at each of the rates in the rateset */
	send_time = 0;
	cw = queue->cw_min;

	int snr = SNR_DEFAULT;

	if (is_multicast_ether_addr(dest)) {
		deststa = NULL;
	} else {
		deststa = get_station_by_addr(ctx, dest);
		if (deststa) {
            w_logf(ctx, LOG_DEBUG, "Packet from " MAC_FMT "(%d|%s) to " MAC_FMT "(%d|%s)\n",
                   MAC_ARGS(station->addr), station->index, station->isap ? "AP" : "Sta",
                   MAC_ARGS(deststa->addr), deststa->index, deststa->isap ? "AP" : "Sta");
            detect_mediums(ctx,station,deststa);
			snr = ctx->get_link_snr(ctx, station, deststa) -
				get_signal_offset_by_interference(ctx,
					station->index, deststa->index);
			snr += ctx->get_fading_signal(ctx);
		}
	}
	frame->signal = snr + NOISE_LEVEL;

	noack = frame_is_mgmt(frame) || is_multicast_ether_addr(dest);
	double choice = -3.14;

	if (use_fixed_random_value(ctx))
		choice = drand48();

	for (i = 0; i < frame->tx_rates_count && !is_acked; i++) {

		rate_idx = frame->tx_rates[i].idx;

		/* no more rates in MRR */
		if (rate_idx < 0)
			break;

		error_prob = ctx->get_error_prob(ctx, snr, rate_idx,
						 frame->freq, frame->data_len,
						 station, deststa);
		for (j = 0; j < frame->tx_rates[i].count; j++) {
			send_time += difs + pkt_duration(ctx, frame->data_len,
				index_to_rate(rate_idx, frame->freq));

			retries++;

			/* skip ack/backoff/retries for noack frames */
			if (noack) {
				is_acked = true;
				break;
			}

			/* TODO TXOPs */

			/* backoff */
			if (j > 0) {
				send_time += (cw * slot_time) / 2;
				cw = (cw << 1) + 1;
				if (cw > queue->cw_max)
					cw = queue->cw_max;
			}
			if (!use_fixed_random_value(ctx))
				choice = drand48();
			if (choice > error_prob) {
				is_acked = true;
				break;
			}
			send_time += ack_time_usec;
		}
	}
	if (is_acked) {
		frame->tx_rates[i-1].count = j + 1;
		for (; i < frame->tx_rates_count; i++) {
			frame->tx_rates[i].idx = -1;
			frame->tx_rates[i].count = -1;
		}
		frame->flags |= HWSIM_TX_STAT_ACK;
	}

	/*
	 * delivery time starts after any equal or higher prio frame
	 * (or now, if none).
	 */
	target = now;
    w_logf(ctx, LOG_DEBUG, "Sta " MAC_FMT " medium is #%d\n", MAC_ARGS(station->addr), station->medium_id);
    list_for_each_entry(tmpsta, &ctx->stations, list) {
        if (station->medium_id == tmpsta->medium_id) {
            w_logf(ctx, LOG_DEBUG, "Sta " MAC_FMT " medium is also #%d\n", MAC_ARGS(tmpsta->addr),
                   tmpsta->medium_id);
            for (i = 0; i <= ac; i++) {
                tail = list_last_entry_or_null(&tmpsta->queues[i].frames,
                                               struct frame, list);
                if (tail && timespec_before(&target, &tail->expires))
                    target = tail->expires;
            }
        } else {
            w_logf(ctx, LOG_DEBUG, "Sta " MAC_FMT " medium is not #%d, it is #%d\n", MAC_ARGS(tmpsta->addr),
                   station->medium_id, tmpsta->medium_id);
        }
    }

	timespec_add_usec(&target, send_time);

	frame->duration = send_time;
	frame->expires = target;
	list_add_tail(&frame->list, &queue->frames);
	rearm_timer(ctx);
}

/*
 * Report transmit status to the kernel.
 */
static int send_tx_info_frame_nl(struct wmediumd *ctx, struct frame *frame)
{
	struct nl_sock *sock = ctx->sock;
	struct nl_msg *msg;
	int ret;

	msg = nlmsg_alloc();
	if (!msg) {
		w_logf(ctx, LOG_ERR, "Error allocating new message MSG!\n");
		return -1;
	}

	if (genlmsg_put(msg, NL_AUTO_PID, NL_AUTO_SEQ, ctx->family_id,
			0, NLM_F_REQUEST, HWSIM_CMD_TX_INFO_FRAME,
			VERSION_NR) == NULL) {
		w_logf(ctx, LOG_ERR, "%s: genlmsg_put failed\n", __func__);
		ret = -1;
		goto out;
	}

	if (nla_put(msg, HWSIM_ATTR_ADDR_TRANSMITTER, ETH_ALEN,
		    frame->sender->hwaddr) ||
	    nla_put_u32(msg, HWSIM_ATTR_FLAGS, frame->flags) ||
	    nla_put_u32(msg, HWSIM_ATTR_SIGNAL, frame->signal) ||
	    nla_put(msg, HWSIM_ATTR_TX_INFO,
		    frame->tx_rates_count * sizeof(struct hwsim_tx_rate),
		    frame->tx_rates) ||
	    nla_put_u64(msg, HWSIM_ATTR_COOKIE, frame->cookie)) {
			w_logf(ctx, LOG_ERR, "%s: Failed to fill a payload\n", __func__);
			ret = -1;
			goto out;
	}

	ret = nl_send_auto_complete(sock, msg);
	if (ret < 0) {
		w_logf(ctx, LOG_ERR, "%s: nl_send_auto failed\n", __func__);
		ret = -1;
		goto out;
	}
	ret = 0;

out:
	nlmsg_free(msg);
	return ret;
}

/*
 * Report transmit status to the transmitter.
 */
static int send_tx_info_frame(struct wmediumd *ctx, struct frame *frame)
{
	if (ctx->op_mode == LOCAL)
		return send_tx_info_frame_nl(ctx, frame);
	
	int numBytes, ret;

	if (is_ap){
		numBytes = sendto(ctx->net_sock, frame, sizeof(*frame), 0, (struct sockaddr*)&clientAddr, len);
        if (numBytes < 0){
			w_flogf(ctx, LOG_ERR, stderr, "Failed to send xmit info to station: %s", strerror(errno));
			ret = -1;
			goto out;
		}
	}
	else{
		numBytes = sendto(ctx->net_sock, frame, sizeof(*frame), 0, (struct sockaddr*)&serverAddr, len);
        if (numBytes < 0){
			w_flogf(ctx, LOG_ERR, stderr, "Failed to send xmit info to AP station: %s", strerror(errno));
			ret = -1;
			goto out;
		}
	}
	ret = 0;

out:
	return ret;
}

/*
 * Send a data frame to the kernel for reception at a specific radio.
 */
int send_cloned_frame_msg(struct wmediumd *ctx, struct station *dst,
			  u8 *data, int data_len, int rate_idx, int signal,
			  int freq)
{
	struct nl_msg *msg;
	struct nl_sock *sock = ctx->sock;
	int ret;

	msg = nlmsg_alloc();
	if (!msg) {
		w_logf(ctx, LOG_ERR, "Error allocating new message MSG!\n");
		return -1;
	}

	if (genlmsg_put(msg, NL_AUTO_PID, NL_AUTO_SEQ, ctx->family_id,
			0, NLM_F_REQUEST, HWSIM_CMD_FRAME,
			VERSION_NR) == NULL) {
		w_logf(ctx, LOG_ERR, "%s: genlmsg_put failed\n", __func__);
		ret = -1;
		goto out;
	}

	if (nla_put(msg, HWSIM_ATTR_ADDR_RECEIVER, ETH_ALEN,
		    dst->hwaddr) ||
	    nla_put(msg, HWSIM_ATTR_FRAME, data_len, data) ||
	    nla_put_u32(msg, HWSIM_ATTR_RX_RATE, rate_idx) ||
	    nla_put_u32(msg, HWSIM_ATTR_FREQ, freq) ||
	    nla_put_u32(msg, HWSIM_ATTR_SIGNAL, signal)) {
			w_logf(ctx, LOG_ERR, "%s: Failed to fill a payload\n", __func__);
			ret = -1;
			goto out;
	}

	w_logf(ctx, LOG_DEBUG, "cloned msg dest " MAC_FMT " (radio: " MAC_FMT ") len %d\n",
		   MAC_ARGS(dst->addr), MAC_ARGS(dst->hwaddr), data_len);

	ret = nl_send_auto_complete(sock, msg);
	if (ret < 0) {
		w_logf(ctx, LOG_ERR, "%s: nl_send_auto failed\n", __func__);
		ret = -1;
		goto out;
	}
	ret = 0;

out:
	nlmsg_free(msg);
	return ret;
}

void deliver_frame(struct wmediumd *ctx, struct frame *frame)
{
	struct ieee80211_hdr *hdr = (void *) frame->data;
	struct station *station;
	u8 *dest = hdr->addr1;
	u8 *src = frame->sender->addr;

	if (frame->flags & HWSIM_TX_STAT_ACK) {
		/* rx the frame on the dest interface */
		list_for_each_entry(station, &ctx->stations, list) {
			if (memcmp(src, station->addr, ETH_ALEN) == 0)
				continue;

			int rate_idx;
			if (is_multicast_ether_addr(dest)) {
				int snr, signal;
				double error_prob;
				/*
				 * we may or may not receive this based on
				 * reverse link from sender -- check for
				 * each receiver.
				 */
				snr = ctx->get_link_snr(ctx, frame->sender,
							station);
				snr += ctx->get_fading_signal(ctx);
				signal = snr + NOISE_LEVEL;
				if (signal < CCA_THRESHOLD)
					continue;

				if (set_interference_duration(ctx,
					frame->sender->index, frame->duration,
					signal))
					continue;

				snr -= get_signal_offset_by_interference(ctx,
					frame->sender->index, station->index);
				rate_idx = frame->tx_rates[0].idx;
				error_prob = ctx->get_error_prob(ctx,
					(double)snr, rate_idx, frame->freq,
					frame->data_len, frame->sender,
					station);

				if (drand48() <= error_prob) {
					w_logf(ctx, LOG_INFO, "Dropped mcast from "
						   MAC_FMT " to " MAC_FMT " at receiver\n",
						   MAC_ARGS(src), MAC_ARGS(station->addr));
					continue;
				}

				send_cloned_frame_msg(ctx, station,
						      frame->data,
						      frame->data_len,
						      rate_idx, signal,
						      frame->freq);
			} else if (memcmp(dest, station->addr, ETH_ALEN) == 0) {
				if (set_interference_duration(ctx,
					frame->sender->index, frame->duration,
					frame->signal))
					continue;
				// use the value that is last in list.
				rate_idx = frame->tx_rates[0].idx;
				for(int i = 0; i < frame->tx_rates_count; i++) {
					if(frame->tx_rates[i].idx == -1) {
						break;
					}
					rate_idx = frame->tx_rates[i].idx;
				}
				send_cloned_frame_msg(ctx, station,
						      frame->data,
						      frame->data_len,
						      rate_idx, frame->signal,
						      frame->freq);
  			}
		}
	} else
		set_interference_duration(ctx, frame->sender->index,
					  frame->duration, frame->signal);

	send_tx_info_frame(ctx, frame);

	free(frame);
}

void deliver_expired_frames_queue(struct wmediumd *ctx,
				  struct list_head *queue,
				  struct timespec *now)
{
	struct frame *frame, *tmp;

	list_for_each_entry_safe(frame, tmp, queue, list) {
		if (timespec_before(&frame->expires, now)) {
			list_del(&frame->list);
			deliver_frame(ctx, frame);
		} else {
			break;
		}
	}
}

void deliver_expired_frames(struct wmediumd *ctx)
{
	struct timespec now, _diff;
	struct station *station;
	struct list_head *l,*rand_it,*rand_start;
    int i, j, duration;
    int sta1_medium_id;

	clock_gettime(CLOCK_MONOTONIC, &now);
	
	int rand_start_cnt = rand() % ctx->num_stas;
	
	// Find the randomized starting point in the list
	rand_start = &ctx->stations;
	for (i = 0; i < rand_start_cnt; i++) {
		rand_start = rand_start->next;
	}

	list_for_each(rand_it, rand_start) {
		//skip if we iterate over head
		if(rand_it == &ctx->stations)
			continue;
		station = list_entry(rand_it, struct station, list);
		int q_ct[IEEE80211_NUM_ACS] = {};
		for (i = 0; i < IEEE80211_NUM_ACS; i++) {
			list_for_each(l, &station->queues[i].frames) {
				q_ct[i]++;
			}
		}
		w_logf(ctx, LOG_DEBUG, "[" TIME_FMT "] Station " MAC_FMT
					   " BK %d BE %d VI %d VO %d\n",
			   TIME_ARGS(&now), MAC_ARGS(station->addr),
			   q_ct[IEEE80211_AC_BK], q_ct[IEEE80211_AC_BE],
			   q_ct[IEEE80211_AC_VI], q_ct[IEEE80211_AC_VO]);

		for (i = 0; i < IEEE80211_NUM_ACS; i++)
			deliver_expired_frames_queue(ctx, &station->queues[i].frames, &now);
	}
	w_logf(ctx, LOG_DEBUG, "\n\n");

	if (!ctx->intf)
		return;

	timespec_sub(&now, &ctx->intf_updated, &_diff);
	duration = (_diff.tv_sec * 1000000) + (_diff.tv_nsec / 1000);
	if (duration < 10000) // calc per 10 msec
		return;

	// update interference
	for (i = 0; i < ctx->num_stas; i++){
        sta1_medium_id = ctx->sta_array[i]->medium_id;
        for (j = 0; j < ctx->num_stas; j++) {
            if (i == j)
                continue;
            if (sta1_medium_id != ctx->sta_array[j]->medium_id)
                continue;
            // probability is used for next calc
            ctx->intf[i * ctx->num_stas + j].prob_col =
                    ctx->intf[i * ctx->num_stas + j].duration /
                    (double)duration;
            ctx->intf[i * ctx->num_stas + j].duration = 0;
        }
    }

	clock_gettime(CLOCK_MONOTONIC, &ctx->intf_updated);
}

static int process_recvd_data(struct wmediumd *ctx, struct nlmsghdr *nlh)
{
	struct nlattr *attrs[HWSIM_ATTR_MAX+1];
	/* generic netlink header*/
	struct genlmsghdr *gnlh = nlmsg_data(nlh);

	struct station *sender;
	struct frame *frame;
	struct ieee80211_hdr *hdr;
	u8 *src;

	if (gnlh->cmd == HWSIM_CMD_FRAME) {
		pthread_rwlock_rdlock(&snr_lock);
		/* we get the attributes*/
		genlmsg_parse(nlh, 0, attrs, HWSIM_ATTR_MAX, NULL);
		if (attrs[HWSIM_ATTR_ADDR_TRANSMITTER]) {
			u8 *hwaddr = (u8 *)nla_data(attrs[HWSIM_ATTR_ADDR_TRANSMITTER]);

			unsigned int data_len =
				nla_len(attrs[HWSIM_ATTR_FRAME]);
			char *data = (char *)nla_data(attrs[HWSIM_ATTR_FRAME]);
			unsigned int flags =
				nla_get_u32(attrs[HWSIM_ATTR_FLAGS]);
			unsigned int tx_rates_len =
				nla_len(attrs[HWSIM_ATTR_TX_INFO]);
			struct hwsim_tx_rate *tx_rates =
				(struct hwsim_tx_rate *)
				nla_data(attrs[HWSIM_ATTR_TX_INFO]);
			u64 cookie = nla_get_u64(attrs[HWSIM_ATTR_COOKIE]);
			u32 freq;
			freq = attrs[HWSIM_ATTR_FREQ] ?
					nla_get_u32(attrs[HWSIM_ATTR_FREQ]) : 2412;

			hdr = (struct ieee80211_hdr *)data;
			src = hdr->addr2;
			
			w_logf(ctx, LOG_DEBUG, "f: %02x%02x d: %02x%02x ",
					(u32)hdr->frame_control[0], (u32)hdr->frame_control[1], (u32)hdr->duration_id[0], (u32)hdr->duration_id[1]);
			
			if (data_len < 6 + 6 + 4)
				goto out;

			sender = get_station_by_addr(ctx, src);
			if (!sender) {
				w_flogf(ctx, LOG_ERR, stderr, "Unable to find sender station " MAC_FMT "\n", MAC_ARGS(src));
				goto out;
			}
			memcpy(sender->hwaddr, hwaddr, ETH_ALEN);

			frame = malloc(sizeof(*frame) + data_len);
			if (!frame)
				goto out;

			memcpy(frame->data, data, data_len);
			frame->data_len = data_len;
			frame->flags = flags;
			frame->cookie = cookie;
			frame->freq = freq;
			frame->sender = sender;
			sender->freq = freq;
			frame->tx_rates_count =
				tx_rates_len / sizeof(struct hwsim_tx_rate);
			memcpy(frame->tx_rates, tx_rates,
			       min(tx_rates_len, sizeof(frame->tx_rates)));
			
			w_logf(ctx, LOG_DEBUG, "a1: " MAC_FMT " a2: " MAC_FMT " a3: " MAC_FMT " sq: %02x%02x r: " MAC_FMT" len: %d cookie: %lld\n", 
					MAC_ARGS(hdr->addr1), MAC_ARGS(hdr->addr2), MAC_ARGS(hdr->addr3), (u32)hdr->seq_ctrl[0], (u32)hdr->seq_ctrl[1], 
					MAC_ARGS(frame->sender->hwaddr), data_len, cookie);
			
			queue_frame(ctx, sender, frame);
		}
out:
		pthread_rwlock_unlock(&snr_lock);
		return 0;

	}
	return 0;
}

static
int nl_err_cb(struct sockaddr_nl *nla, struct nlmsgerr *nlerr, void *arg)
{
	struct genlmsghdr *gnlh = nlmsg_data(&nlerr->msg);
	struct wmediumd *ctx = arg;

	w_flogf(ctx, LOG_ERR, stderr, "nl: cmd %d, seq %d: %s\n", gnlh->cmd,
			nlerr->msg.nlmsg_seq, strerror(abs(nlerr->error)));

	return NL_SKIP;
}

struct frame* construct_tx_info_frame(struct wmediumd *ctx, struct nlmsghdr *nlh)
{
	struct nlattr *attrs[HWSIM_ATTR_MAX+1];
	struct genlmsghdr *gnlh = nlmsg_data(nlh);

	struct station *sender;
	struct frame *frame = NULL;
	struct ieee80211_hdr *hdr;

	if (gnlh->cmd == HWSIM_CMD_FRAME){
		genlmsg_parse(nlh, 0, attrs, HWSIM_ATTR_MAX, NULL);
		u8 *hwaddr = (u8 *)nla_data(attrs[HWSIM_ATTR_ADDR_TRANSMITTER]);
		u64 cookie = nla_get_u64(attrs[HWSIM_ATTR_COOKIE]);
		char *data = (char *)nla_data(attrs[HWSIM_ATTR_FRAME]);

		hdr = (struct ieee80211_hdr *)data;

		sender = get_station_by_addr(ctx, hdr->addr2);
		if (!sender) {
			w_flogf(ctx, LOG_ERR, stderr, "%s: Unable to find sender station " MAC_FMT "\n", __FUNCTION__, MAC_ARGS(hdr->addr2));
			goto out;
		}
		memcpy(sender->hwaddr, hwaddr, ETH_ALEN);
		
		frame = malloc(sizeof(struct frame));
		if (!frame)
			goto out;
		
		frame->cookie = cookie;
		frame->sender = sender;
	}

out:
	return frame;
}

/*
 * Handle events from the kernel.  Process CMD_FRAME events and queue them
 * for later delivery with the scheduler.
 */
static int process_messages_cb(struct nl_msg *msg, void *arg)
{
	struct nlmsghdr *nlh = nlmsg_hdr(msg);
	struct wmediumd* ctx = (struct wmediumd*)arg;
	struct genlmsghdr *gnlh = nlmsg_data(nlh);

	/* PMSR (FTM) is answered locally by the medium, regardless of op_mode. */
	if (kernel_supports_pmsr()) {
		if (gnlh->cmd == HWSIM_CMD_START_PMSR) {
			hwsim_handle_pmsr_request(ctx, nlh);
			return 0;
		}
		if (gnlh->cmd == HWSIM_CMD_ABORT_PMSR)
			return 0;
	}

	if (ctx->op_mode == LOCAL)
		return process_recvd_data(ctx, nlh);
	
	int numBytes, ret;

	out_buf = malloc(nlh->nlmsg_len);
	memcpy(out_buf, nlh, nlh->nlmsg_len);

	if (is_ap){
		numBytes = sendto(ctx->net_sock, out_buf, nlh->nlmsg_len, 0, (struct sockaddr*)&clientAddr, len);
        if (numBytes < 0){
			w_flogf(ctx, LOG_ERR, stderr, "Failed to send data to station: %s", strerror(errno));
			ret = -1;
			goto out;
		}
	}
	else{
		numBytes = sendto(ctx->net_sock, out_buf, nlh->nlmsg_len, 0, (struct sockaddr*)&serverAddr, len);
        if (numBytes < 0){
			w_flogf(ctx, LOG_ERR, stderr, "Failed to send data to AP station: %s", strerror(errno));
			ret = -1;
			goto out;
		}
	}
	ret = 0;
	struct frame* tx_frame = construct_tx_info_frame(ctx, nlh);
	if (tx_frame != NULL)
		list_add_tail(&tx_frame->list, &ctx->pending_txinfo_frames);

out:
	free(out_buf);
	return ret;
}

/*
 * Register with the kernel to start receiving new frames.
 */
int send_register_msg(struct wmediumd *ctx)
{
	struct nl_sock *sock = ctx->sock;
	struct nl_msg *msg;
	int ret;

	msg = nlmsg_alloc();
	if (!msg) {
		w_logf(ctx, LOG_ERR, "Error allocating new message MSG!\n");
		return -1;
	}

	if (genlmsg_put(msg, NL_AUTO_PID, NL_AUTO_SEQ, ctx->family_id,
			0, NLM_F_REQUEST, HWSIM_CMD_REGISTER,
			VERSION_NR) == NULL) {
		w_logf(ctx, LOG_ERR, "%s: genlmsg_put failed\n", __func__);
		ret = -1;
		goto out;
	}

	ret = nl_send_auto_complete(sock, msg);
	if (ret < 0) {
		w_logf(ctx, LOG_ERR, "%s: nl_send_auto failed\n", __func__);
		ret = -1;
		goto out;
	}
	ret = 0;

out:
	nlmsg_free(msg);
	return ret;
}

static void net_sock_event_cb(int fd, short what, void *data)
{
	struct wmediumd *ctx = data;
	int numBytes;
	struct frame* pending_frame;
	struct frame tx_info_frame;
	bzero(in_buf, PAGE_SIZE);

	if (is_ap){
		numBytes = recvfrom(fd, in_buf, PAGE_SIZE, 0, (struct sockaddr*)&clientAddr, &len);
        if (numBytes < 0){
			w_flogf(ctx, LOG_ERR, stderr, "Failed to receive data from station: %s", strerror(errno));
			return;
		}
	}
	else{
		numBytes = recvfrom(fd, in_buf, PAGE_SIZE, 0, (struct sockaddr*)&serverAddr, &len);
        if (numBytes < 0){
			w_flogf(ctx, LOG_ERR, stderr, "Failed to receive data from AP station: %s", strerror(errno));
			return;
		}
	}
	/* First check if the received message is for transmit status */
	memcpy(&tx_info_frame, in_buf, sizeof(struct frame));
	list_for_each_entry(pending_frame, &ctx->pending_txinfo_frames, list)
	{
		if (pending_frame->cookie == tx_info_frame.cookie){
			pending_frame->flags = tx_info_frame.flags;
			pending_frame->signal = tx_info_frame.signal;
			pending_frame->tx_rates_count = tx_info_frame.tx_rates_count;
			memcpy(pending_frame->tx_rates, tx_info_frame.tx_rates, 
					pending_frame->tx_rates_count * sizeof(struct hwsim_tx_rate));
			send_tx_info_frame_nl(ctx, pending_frame);
			list_del(&pending_frame->list);
			free(pending_frame);
			return;
		}
	}
	process_recvd_data(ctx, (struct nlmsghdr*)in_buf);
}

static void sock_event_cb(int fd, short what, void *data)
{
	struct wmediumd *ctx = data;

	nl_recvmsgs_default(ctx->sock);
}

static struct event ev_reregister;
static struct timeval reregister_iv = { 2, 0 };

/*
 * Periodically re-assert the hwsim registration. Under Mininet-WiFi the
 * kernel can drop wmediumd's medium registration mid-setup (it is released
 * when the position/config client disconnects from the wserver), silently
 * switching hwsim to the perfect channel and disabling PMSR/FTM. The netlink
 * socket stays open, so re-sending HWSIM_CMD_REGISTER restores the medium;
 * the kernel answers -EBUSY while a registration is still in place.
 */
static void reregister_cb(int fd, short what, void *data)
{
	struct wmediumd *ctx = data;

	send_register_msg(ctx);
	evtimer_add(&ev_reregister, &reregister_iv);
}

/*
 * Setup netlink socket and callbacks.
 */
static int init_netlink(struct wmediumd *ctx)
{
	struct nl_sock *sock;
	int ret;

	ctx->cb = nl_cb_alloc(NL_CB_CUSTOM);
	if (!ctx->cb) {
		w_logf(ctx, LOG_ERR, "Error allocating netlink callbacks\n");
		return -1;
	}

	sock = nl_socket_alloc_cb(ctx->cb);
	if (!sock) {
		w_logf(ctx, LOG_ERR, "Error allocating netlink socket\n");
		return -1;
	}

	ctx->sock = sock;

	ret = genl_connect(sock);
	if (ret < 0) {
		w_logf(ctx, LOG_ERR, "Error connecting netlink socket ret=%d\n", ret);
		return -1;
	}

	ctx->family_id = genl_ctrl_resolve(sock, "MAC80211_HWSIM");
	if (ctx->family_id < 0) {
		w_logf(ctx, LOG_ERR, "Family MAC80211_HWSIM not registered\n");
		return -1;
	}

	nl_cb_set(ctx->cb, NL_CB_MSG_IN, NL_CB_CUSTOM, process_messages_cb, ctx);
	nl_cb_err(ctx->cb, NL_CB_CUSTOM, nl_err_cb, ctx);

	return 0;
}

int init_remote_connection(struct wmediumd* ctx, char* ap_ipaddr)
{
	int numBytes, ret = 0;

	ctx->net_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (ctx->net_sock < 0){
		w_flogf(ctx, LOG_ERR, stderr, "Failed to open socket: %s\n", strerror(errno));
		ret = -1;
		goto out;
	}
    
	bzero(&serverAddr, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(AP_DEFAULT_PORT);
	len = sizeof(struct sockaddr_in);

	/* AP server configuration */
	if (ap_ipaddr == NULL){
		serverAddr.sin_addr.s_addr = INADDR_ANY;
		if (bind(ctx->net_sock, (struct sockaddr *)&serverAddr, len) < 0){
			w_flogf(ctx, LOG_ERR, stderr, "Failed to bind socket: %s\n", strerror(errno));
			ret = -1;
			goto out;
		}
		w_logf(ctx, LOG_INFO, "Waiting for station machine to connect\n");
		
		/* Wait until station machine connects. This is primarily meant to get the client details */
		numBytes = recvfrom(ctx->net_sock, in_buf, 5, 0, (struct sockaddr*)&clientAddr, &len);
        if (numBytes < 0){
			w_flogf(ctx, LOG_ERR, stderr, "Failed to receive initiate command from client: %s\n", strerror(errno));
			ret = -1;
			goto out;
		}
		w_logf(ctx, LOG_NOTICE, "Station machine connected successfully\n");
	}
	else{
		struct hostent *server = NULL;
		server = gethostbyname(ap_ipaddr);
		if (server == NULL){
			w_flogf(ctx, LOG_ERR, stderr, "No host with the given hostname exists\n");
			ret = -1;
			goto out;
		}
		bcopy((char*)server->h_addr_list[0], (char*)&serverAddr.sin_addr.s_addr, server->h_length);
		
		/* The client must send a message first for the server to know the client address details */
		const char* initiate_comm = "start";
		numBytes = sendto(ctx->net_sock, initiate_comm, strlen(initiate_comm), 0, (const struct sockaddr*)&serverAddr, len);
		if (numBytes < 0){
			w_flogf(ctx, LOG_ERR, stderr, "Failed to send initiate message: %s\n", strerror(errno));
			ret = -1;
			goto out;
		}
		w_logf(ctx, LOG_INFO, "Initiate communication command sent to access point machine\n");
	}

out:
	return ret;
}

/*
 *	Print the CLI help
 */
void print_help(int exval)
{
	printf("wmediumd v%s - a wireless medium simulator\n", VERSION_STR);
	printf("wmediumd [-h] [-V] [-a AP_ADDR] [-s] [-l LOG_LVL] [-x FILE] -c FILE\n\n");

	printf("  -h              print this help and exit\n");
	printf("  -V              print version and exit\n\n");

	printf("  -l LOG_LVL      set the logging level\n");
	printf("                  LOG_LVL: RFC 5424 severity, values 0 - 7\n");
	printf("                  >= 3: errors are logged\n");
	printf("                  >= 5: startup msgs are logged\n");
	printf("                  >= 6: dropped packets are logged (default)\n");
	printf("                  == 7: all packets will be logged\n");
	printf("  -c FILE         set input config file\n");
	printf("  -x FILE         set input PER file\n");
	printf("  -s              start the server on a socket\n");
	printf("  -d              use the dynamic complex mode\n");
	printf("                  (server only with matrices for each connection)\n");
	printf("  -a AP_ADDR      Set remote operation mode to associate remotely\n");
	printf("                  AP_ADDR: IPv4 address or hostname of AP machine\n");
	printf("                  \"localhost\" if current machine is AP\n");

	exit(exval);
}

static void timer_cb(int fd, short what, void *data)
{
	struct wmediumd *ctx = data;
	uint64_t u;

	pthread_rwlock_rdlock(&snr_lock);
	read(fd, &u, sizeof(u));
	ctx->move_stations(ctx);
	deliver_expired_frames(ctx);
	rearm_timer(ctx);
	pthread_rwlock_unlock(&snr_lock);
}

int main(int argc, char *argv[])
{
	int opt;
	struct event ev_cmd, net_ev;
	struct event ev_timer;
	struct wmediumd ctx;
	char *config_file = NULL;
	char *per_file = NULL;
	char* ap_ip = NULL;

	setvbuf(stdout, NULL, _IOLBF, BUFSIZ);

	if (argc == 1) {
		fprintf(stderr, "This program needs arguments....\n\n");
		print_help(EXIT_FAILURE);
	}

	ctx.log_lvl = 6;
	unsigned long int parse_log_lvl;
	char* parse_end_token;
	bool start_server = false;
	bool full_dynamic = false;

	while ((opt = getopt(argc, argv, "hVc:l:x:sda:")) != -1) {
		switch (opt) {
		case 'h':
			print_help(EXIT_SUCCESS);
			break;
		case 'V':
			printf("wmediumd v%s - a wireless medium simulator "
			       "for mac80211_hwsim\n", VERSION_STR);
			exit(EXIT_SUCCESS);
			break;
		case 'c':
			config_file = optarg;
			break;
		case 'x':
			printf("Input packet error rate file: %s\n", optarg);
			per_file = optarg;
			break;
		case ':':
			printf("wmediumd: Error - Option `%c' "
			       "needs a value\n\n", optopt);
			print_help(EXIT_FAILURE);
			break;
		case 'l':
			parse_log_lvl = strtoul(optarg, &parse_end_token, 10);
			if ((parse_log_lvl == ULONG_MAX && errno == ERANGE) ||
			     optarg == parse_end_token || parse_log_lvl > 7) {
				printf("wmediumd: Error - Invalid RFC 5424 severity level: "
							   "%s\n\n", optarg);
				print_help(EXIT_FAILURE);
			}
			ctx.log_lvl = parse_log_lvl;
			break;
		case 'd':
			full_dynamic = true;
			break;
		case 's':
			start_server = true;
			break;
		case 'a':
			ap_ip = optarg;
			ctx.op_mode = REMOTE;
			is_ap = strcmp(ap_ip, "localhost") == 0;
			break;
		case '?':
			printf("wmediumd: Error - No such option: "
			       "`%c'\n\n", optopt);
			print_help(EXIT_FAILURE);
			break;
		}

	}

	if (optind < argc)
		print_help(EXIT_FAILURE);

	if (ap_ip == NULL)
		ctx.op_mode = LOCAL;

	if (full_dynamic) {
		if (config_file) {
			printf("%s: cannot use dynamic complex mode with config file\n", argv[0]);
			print_help(EXIT_FAILURE);
		}

		if (!start_server) {
			printf("%s: dynamic complex mode requires the server option\n", argv[0]);
			print_help(EXIT_FAILURE);
		}

		w_logf(&ctx, LOG_NOTICE, "Using dynamic complex mode instead of config file\n");
	} else {
		if (!config_file) {
			printf("%s: config file must be supplied\n", argv[0]);
			print_help(EXIT_FAILURE);
		}

		w_logf(&ctx, LOG_NOTICE, "Input configuration file: %s\n", config_file);
	}
	INIT_LIST_HEAD(&ctx.stations);
	if (load_config(&ctx, config_file, per_file, full_dynamic))
		return EXIT_FAILURE;

	/* init libevent */
	event_init();

	if (ctx.op_mode == REMOTE){
		INIT_LIST_HEAD(&ctx.pending_txinfo_frames);
		if (init_remote_connection(&ctx, is_ap ? NULL : ap_ip) < 0)
			return EXIT_FAILURE;
		event_set(&net_ev, ctx.net_sock, EV_READ | EV_PERSIST, net_sock_event_cb, &ctx);
		event_add(&net_ev, NULL);
	}

	/* init netlink */
	if (init_netlink(&ctx) < 0)
		return EXIT_FAILURE;

	event_set(&ev_cmd, nl_socket_get_fd(ctx.sock), EV_READ | EV_PERSIST,
		  sock_event_cb, &ctx);
	event_add(&ev_cmd, NULL);

	/* setup timers */
	ctx.timerfd = timerfd_create(CLOCK_MONOTONIC, 0);
	clock_gettime(CLOCK_MONOTONIC, &ctx.intf_updated);
	clock_gettime(CLOCK_MONOTONIC, &ctx.next_move);
	ctx.next_move.tv_sec += MOVE_INTERVAL;
	event_set(&ev_timer, ctx.timerfd, EV_READ | EV_PERSIST, timer_cb, &ctx);
	event_add(&ev_timer, NULL);

	/* register for new frames */
	if (send_register_msg(&ctx) == 0) {
		w_logf(&ctx, LOG_NOTICE, "REGISTER SENT!\n");
	}

	/* keep the registration alive across spurious kernel releases */
	evtimer_set(&ev_reregister, reregister_cb, &ctx);
	evtimer_add(&ev_reregister, &reregister_iv);

	if (start_server == true)
		start_wserver(&ctx);

	/* enter libevent main loop */
	event_dispatch();

	if (start_server == true)
		stop_wserver();

	free(ctx.sock);
	free(ctx.cb);
	free(ctx.intf);
	free(ctx.per_matrix);

	return EXIT_SUCCESS;
}
