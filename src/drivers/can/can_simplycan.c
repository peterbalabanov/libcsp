/*
Cubesat Space Protocol - A small network-layer protocol designed for Cubesats
Copyright (C) 2012 GomSpace ApS (http://www.gomspace.com)
Copyright (C) 2012 AAUSAT3 Project (http://aausat3.space.aau.dk)

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <csp/drivers/can_simplycan.h>

#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <csp/drivers/simply.h>
#include <sys/time.h>

#include <csp/csp.h>
#include <csp/arch/csp_thread.h>

// CAN interface data, state, etc.
typedef struct {
    char name[CSP_IFLIST_NAME_MAX + 1];
    csp_iface_t iface;
    csp_can_interface_data_t ifdata;
    pthread_t rx_thread;
} can_context_t;

/* special address description flags for the CAN_ID */
#define CAN_EFF_FLAG 0x80000000U /* EFF/SFF is set in the MSB */
#define CAN_RTR_FLAG 0x40000000U /* remote transmission request */
#define CAN_ERR_FLAG 0x20000000U /* error frame */

/* valid bits in CAN ID for frame formats */
#define CAN_SFF_MASK 0x000007FFU /* standard frame format (SFF) */
#define CAN_EFF_MASK 0x1FFFFFFFU /* extended frame format (EFF) */
#define CAN_ERR_MASK 0x1FFFFFFFU /* omit EFF, RTR, ERR flags */

static bool status(void);

static void simplycan_free(can_context_t *ctx) {
    simply_stop_can();
    simply_close();
    if (ctx) {
        free(ctx);
    }
}

static void *simplycan_rx_thread(void *arg) {
    can_context_t *ctx = arg;
    can_msg_t can_msg;

    while (1) {
        usleep(20000); /* sleep to avoid busy loop */
        /* Read CAN frame */
        memset(&can_msg, 0, sizeof(can_msg));
        int result = simply_receive(&can_msg);
        if (result == -1) {
            csp_log_error("%s[%s]: read() failed, simplycan errorcode:%d", __FUNCTION__, ctx->name,
                          simply_get_last_error());
            continue;
        } else if (result == 0)
            continue;

        /* Drop frames with standard id (CSP uses extended) */
        if (!(can_msg.ident & CAN_EFF_FLAG)) {
            continue;
        }

        /* Drop error and remote frames */
        if (can_msg.ident & (CAN_ERR_FLAG | CAN_RTR_FLAG)) {
            csp_log_warn("%s[%s]: discarding ERR/RTR/SFF frame", __FUNCTION__, ctx->name);
            continue;
        }

        /* Strip flags */
        can_msg.ident &= CAN_EFF_MASK;

        /* Call RX callbacsp_can_rx_frameck */
        csp_can_rx(&ctx->iface, can_msg.ident, can_msg.payload, can_msg.dlc, NULL);
    }

    /* We should never reach this point */
    pthread_exit(NULL);
    return NULL;
}

static int csp_can_tx_frame(void *driver_data, uint32_t id, const uint8_t *data, uint8_t dlc) {
    if (dlc > 8) {
        return CSP_ERR_INVAL;
    }
    can_msg_t can_msg = {.ident = id | CAN_EFF_FLAG, .dlc = dlc};
    memcpy(can_msg.payload, data, dlc);
    struct timeval tp;
    gettimeofday(&tp, NULL);
    can_msg.timestamp = tp.tv_sec * 1000 + tp.tv_usec / 1000;

    uint32_t elapsed_ms = 0;
    can_context_t *ctx = driver_data;
    can_sts_t canstatus = {.sts = 0, .tx_free = 0};

    if (!simply_send(&can_msg)) {
        csp_log_error("simplycan error in tx: %d", simply_get_last_error());
        simplycan_free(ctx);
        return CSP_ERR_TX;
    }

    while (canstatus.sts & CAN_STATUS_PENDING) {
        if (!simply_can_status(&canstatus)) {
            csp_log_error("simplycan error in status petition: %d", simply_get_last_error());
            simplycan_free(ctx);
            return CSP_ERR_TX;
        }
        // status();
        if (elapsed_ms >= 1000) {
            csp_log_warn("%s[%s]: write() failed", __FUNCTION__, ctx->name);
            return CSP_ERR_TX;
        }
        csp_sleep_ms(5);
        elapsed_ms += 5;
    }

    return CSP_ERR_NONE;
}

int csp_can_simplycan_open_and_add_interface(const char *device, const char *ifname, int bitrate, bool promisc,
                                             csp_iface_t **return_iface) {
    if (ifname == NULL) {
        ifname = CSP_IF_CAN_DEFAULT_NAME;
    }

    csp_log_info("INIT %s: device: [%s], bitrate: %d, promisc: %d", ifname, device, bitrate, promisc);

    /* Set interface up - this may require increased OS privileges */
    if (bitrate > 0) {
        if (!simply_open((char *)device)) {
            csp_log_error("simplycan error in %s, line %d: %d", __FUNCTION__, __LINE__, simply_get_last_error());
            simply_close();
            return CSP_ERR_DRIVER;
        }

        identification_t simplycanID;
        memset(&simplycanID, 0, sizeof(simplycanID));

        if (!simply_identify(&simplycanID)) {
            csp_log_error("simplycan error in %s, line %d: %d", __FUNCTION__, __LINE__, simply_get_last_error());
            simply_close();
            return CSP_ERR_DRIVER;
        } else {
            csp_log_info(
                "CAN controller identify:\nFW ver:%s\nHW ver:%s\nProduct string:%s\nProduct ver:%s\nSerial number:%s\n",
                (char *)simplycanID.fw_version, (char *)simplycanID.hw_version, (char *)simplycanID.product_string,
                (char *)simplycanID.product_version, (char *)simplycanID.serial_number);
        }

        if (!simply_initialize_can(bitrate)) {
            csp_log_error("simplycan error in %s, line %d: %d", __FUNCTION__, __LINE__, simply_get_last_error());
            simply_close();
            return CSP_ERR_DRIVER;
        }
    } else {
        csp_log_error("Invalid CAN bitrate");
        return CSP_ERR_INVAL;
    }

    can_context_t *ctx = calloc(1, sizeof(*ctx));
    if (ctx == NULL) {
        return CSP_ERR_NOMEM;
    }

    strncpy(ctx->name, ifname, sizeof(ctx->name) - 1);
    ctx->iface.name = ctx->name;
    ctx->iface.interface_data = &ctx->ifdata;
    ctx->iface.driver_data = ctx;
    ctx->ifdata.tx_func = csp_can_tx_frame;

    /* Set filter mode */
    if (promisc == false) {
        if (!simply_set_filter(CFP_MAKE_DST((1 << CFP_HOST_SIZE) - 1), CFP_MAKE_DST(csp_get_address()))) {
            csp_log_error("simplycan error in %s, line %d: %d", __FUNCTION__, __LINE__, simply_get_last_error());
            simplycan_free(ctx);
            return CSP_ERR_DRIVER;
        }
    }

    if (!simply_start_can()) {
        csp_log_error("simplycan error in %s, line %d: %d", __FUNCTION__, __LINE__, simply_get_last_error());
        simply_close();
        return CSP_ERR_DRIVER;
    }

    /* Add interface to CSP */
    int res = csp_can_add_interface(&ctx->iface);
    if (res != CSP_ERR_NONE) {
        csp_log_error("%s[%s]: csp_can_add_interface() failed, error: %d", __FUNCTION__, ctx->name, res);
        simplycan_free(ctx);
        return res;
        return CSP_ERR_DRIVER;
    }

    /* Create receive thread */
    if (pthread_create(&ctx->rx_thread, NULL, simplycan_rx_thread, ctx) != 0) {
        csp_log_error("%s[%s]: pthread_create() failed, error: %s", __FUNCTION__, ctx->name, strerror(errno));
        // simplycan_free(ctx); // we already added it to CSP (no way to remove it)
        return CSP_ERR_NOMEM;
    }

    if (return_iface) {
        *return_iface = &ctx->iface;
    }

    return CSP_ERR_NONE;
}

csp_iface_t *csp_can_simplycan_init(const char *device, int bitrate, bool promisc) {
    csp_iface_t *return_iface;
    int res =
        csp_can_simplycan_open_and_add_interface(device, CSP_IF_CAN_DEFAULT_NAME, bitrate, promisc, &return_iface);
    return (res == CSP_ERR_NONE) ? return_iface : NULL;
}

int csp_can_simplycan_stop(csp_iface_t *iface) {
    can_context_t *ctx = iface->driver_data;

    int error = pthread_cancel(ctx->rx_thread);
    if (error != 0) {
        csp_log_error("%s[%s]: pthread_cancel() failed, error: %s", __FUNCTION__, ctx->name, strerror(errno));
        return CSP_ERR_DRIVER;
    }
    error = pthread_join(ctx->rx_thread, NULL);
    if (error != 0) {
        csp_log_error("%s[%s]: pthread_join() failed, error: %s", __FUNCTION__, ctx->name, strerror(errno));
        return CSP_ERR_DRIVER;
    }
    simplycan_free(ctx);
    return CSP_ERR_NONE;
}

static bool status(void) {
    can_sts_t can_sts;

    /* format and print CAN status */
    if (!simply_can_status(&can_sts)) {
        return false;
    }
    printf("CAN status: ");
    if (can_sts.sts & CAN_STATUS_RUNNING) {
        printf("--- ");
    }
    if (can_sts.sts & CAN_STATUS_RESET) {
        printf("RST ");
    }

    if (can_sts.sts & CAN_STATUS_BUSOFF) {
        printf("BOF ");
    } else {
        printf("--- ");
    }

    if (can_sts.sts & CAN_STATUS_ERRORSTATUS) {
        printf("ERR ");
    } else {
        printf("--- ");
    }

    if (can_sts.sts & CAN_STATUS_RXOVERRUN) {
        printf("RxO ");
    } else {
        printf("--- ");
    }

    if (can_sts.sts & CAN_STATUS_TXOVERRUN) {
        printf("TxO ");
    } else {
        printf("--- ");
    }

    if (can_sts.sts & CAN_STATUS_PENDING) {
        printf("PDG ");
    } else {
        printf("--- ");
    }
    printf("\n");

    return true;
}