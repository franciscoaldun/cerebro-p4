// USB 2.0 High-Speed (puerto HUSB): el P4 aparece en Windows como
//   1) una webcam UVC con dos formatos (MJPEG y H.264) y
//   2) una tarjeta de red NCM (IP 192.168.7.1, el PC recibe 192.168.7.2 sin puerta de enlace).
#include <string.h>
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "dhcpserver/dhcpserver.h"
#include "esp_timer.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_net.h"
#include "device/usbd_pvt.h"
#include "class/video/video.h"
#include "app.h"

static const char *TAG = "usb";

// ============================ Descriptores ============================
enum { ITF_VC = 0, ITF_VS, ITF_NCM, ITF_NCM_DATA, ITF_TOTAL };
enum { STR_LANG = 0, STR_MANUF, STR_PRODUCT, STR_SERIAL, STR_NET, STR_MAC, STR_CAM, STR_COUNT };

#define EP_VIDEO    0x81
#define EP_NOTIF    0x82
#define EP_NET_OUT  0x03
#define EP_NET_IN   0x83

#define ENT_CAM     1
#define ENT_OUT     2
#define UVC_FPS     30
#define UVC_FI      (10000000 / UVC_FPS)

#define FRM_MJPEG_LEN  (TUD_VIDEO_DESC_CS_VS_FRM_MJPEG_DISC_LEN + 4)          // 1 intervalo discreto
#define FRM_H264_LEN   TUD_VIDEO_DESC_CS_VS_FRM_FRAME_BASED_CONT_LEN            // rango continuo min=max
#define VS_FORMATS_LEN ( \
    TUD_VIDEO_DESC_CS_VS_FMT_MJPEG_LEN + FRM_MJPEG_LEN + TUD_VIDEO_DESC_CS_VS_COLOR_MATCHING_LEN + \
    TUD_VIDEO_DESC_CS_VS_FMT_FRAME_BASED_LEN + FRM_H264_LEN + TUD_VIDEO_DESC_CS_VS_COLOR_MATCHING_LEN)
#define VS_IN_HDR_LEN  (TUD_VIDEO_DESC_CS_VS_IN_LEN + 2)   // 2 formatos x bControlSize(1)

#define UVC_DESC_LEN ( TUD_VIDEO_DESC_IAD_LEN + TUD_VIDEO_DESC_STD_VC_LEN + (TUD_VIDEO_DESC_CS_VC_LEN + 1) \
    + TUD_VIDEO_DESC_CAMERA_TERM_LEN + TUD_VIDEO_DESC_OUTPUT_TERM_LEN \
    + TUD_VIDEO_DESC_STD_VS_LEN + VS_IN_HDR_LEN + VS_FORMATS_LEN + 7 )
#define CFG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + UVC_DESC_LEN + TUD_CDC_NCM_DESC_LEN)

#define BR(w, h, fps) ((w) * (h) * 16 * (fps))

// Ojo: las macros *_DISC de TinyUSB 0.21 emiten los intervalos como bytes sueltos, y su lectura
// de los descriptores "frame based" usa el formato de los MJPEG. Por eso el MJPEG se arma a mano y
// el H.264 se declara como rango continuo con min = max: así la lectura de TinyUSB igual cae bien.
#define FRAME_MJPEG(idx, w, h)     FRM_MJPEG_LEN, TUSB_DESC_CS_INTERFACE, VIDEO_CS_ITF_VS_FRAME_MJPEG, idx, 0,     U16_TO_U8S_LE(w), U16_TO_U8S_LE(h), U32_TO_U8S_LE(BR(w, h, 5)), U32_TO_U8S_LE(BR(w, h, UVC_FPS)),     U32_TO_U8S_LE((w) * (h) * 2), U32_TO_U8S_LE(UVC_FI), 1, U32_TO_U8S_LE(UVC_FI)
#define FRAME_H264(idx, w, h)     TUD_VIDEO_DESC_CS_VS_FRM_FRAME_BASED_CONT(idx, 0, w, h, 1000000, 40000000, UVC_FI, 0, UVC_FI, UVC_FI, UVC_FI)

#define CONFIG_DESC(_bulk) \
    TUD_CONFIG_DESCRIPTOR(1, ITF_TOTAL, 0, CFG_TOTAL_LEN, 0, 500), \
    /* ---------- UVC ---------- */ \
    TUD_VIDEO_DESC_IAD(ITF_VC, 2, STR_CAM), \
    TUD_VIDEO_DESC_STD_VC(ITF_VC, 0, STR_CAM), \
      TUD_VIDEO_DESC_CS_VC(0x0150, TUD_VIDEO_DESC_CAMERA_TERM_LEN + TUD_VIDEO_DESC_OUTPUT_TERM_LEN, 27000000, ITF_VS), \
        TUD_VIDEO_DESC_CAMERA_TERM(ENT_CAM, 0, 0, 0, 0, 0, 0), \
        TUD_VIDEO_DESC_OUTPUT_TERM(ENT_OUT, VIDEO_TT_STREAMING, 0, ENT_CAM, 0), \
    TUD_VIDEO_DESC_STD_VS(ITF_VS, 0, 1, STR_CAM), \
      VS_IN_HDR_LEN, TUSB_DESC_CS_INTERFACE, VIDEO_CS_ITF_VS_INPUT_HEADER, 2, \
      U16_TO_U8S_LE(VS_IN_HDR_LEN + VS_FORMATS_LEN), EP_VIDEO, 0, ENT_OUT, 0, 0, 0, 1, 0, 0, \
        TUD_VIDEO_DESC_CS_VS_FMT_MJPEG(1, 1, 0, 1, 16, 9, 0, 0), \
          FRAME_MJPEG(1, 1920, 1080), \
          TUD_VIDEO_DESC_CS_VS_COLOR_MATCHING(VIDEO_COLOR_PRIMARIES_BT709, VIDEO_COLOR_XFER_CH_BT709, VIDEO_COLOR_COEF_SMPTE170M), \
        TUD_VIDEO_DESC_CS_VS_FMT_FRAME_BASED(2, 1, TUD_VIDEO_GUID_H264, 16, 1, 16, 9, 0, 0, 1), \
          FRAME_H264(1, 1920, 1080), \
          TUD_VIDEO_DESC_CS_VS_COLOR_MATCHING(VIDEO_COLOR_PRIMARIES_BT709, VIDEO_COLOR_XFER_CH_BT709, VIDEO_COLOR_COEF_SMPTE170M), \
      TUD_VIDEO_DESC_EP_BULK(EP_VIDEO, _bulk, 1), \
    /* ---------- NCM ---------- */ \
    TUD_CDC_NCM_DESCRIPTOR(ITF_NCM, STR_NET, STR_MAC, EP_NOTIF, 64, EP_NET_OUT, EP_NET_IN, _bulk, CFG_TUD_NET_MTU)

static const uint8_t s_cfg_hs[] = { CONFIG_DESC(512) };
static const uint8_t s_cfg_fs[] = { CONFIG_DESC(64) };
_Static_assert(sizeof(s_cfg_hs) == CFG_TOTAL_LEN, "largo del descriptor HS");

static const tusb_desc_device_t s_dev = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0x303A,
    .idProduct = 0x80A4,
    .bcdDevice = 0x0102,          // subir si cambian los descriptores (Windows los guarda en caché)
    .iManufacturer = STR_MANUF,
    .iProduct = STR_PRODUCT,
    .iSerialNumber = STR_SERIAL,
    .bNumConfigurations = 1,
};

static const tusb_desc_device_qualifier_t s_qual = {
    .bLength = sizeof(tusb_desc_device_qualifier_t),
    .bDescriptorType = TUSB_DESC_DEVICE_QUALIFIER,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .bNumConfigurations = 1,
    .bReserved = 0,
};

static char s_serial[16];
static const char *s_str[STR_COUNT] = {
    (const char[]){ 0x09, 0x04 },
    "Aldunate Labs",
    "Cerebro P4",
    s_serial,
    "Red Cerebro P4",
    "",                            // MAC: lo rellena tinyusb_net_init
    "Camara Cerebro P4",
};

// ============================ UVC ============================
static volatile int      s_fmt = 1, s_frm = 1;
static volatile bool     s_busy;
static volatile bool     s_committed;
static const uint8_t    *s_tx_buf;
static size_t            s_tx_len;
static int64_t           s_tx_t0;
static volatile uint32_t s_frames;

int tud_video_commit_cb(uint_fast8_t ctl_idx, uint_fast8_t stm_idx, video_probe_and_commit_control_t const *p)
{
    s_fmt = p->bFormatIndex;
    s_frm = p->bFrameIndex;
    s_busy = false;
    s_committed = true;
    ESP_LOGI(TAG, "UVC: el PC abrió la cámara (%s 1920x1080)", s_fmt == 2 ? "H.264" : "MJPEG");
    return VIDEO_ERROR_NONE;
}

void tud_video_frame_xfer_complete_cb(uint_fast8_t ctl_idx, uint_fast8_t stm_idx)
{
    stats_busy(ENG_USB, s_tx_t0, esp_timer_get_time());
    s_frames++;
    s_busy = false;
}

static void do_uvc_xfer(void *arg)
{
    if (!tud_video_n_frame_xfer(0, 0, (void *)s_tx_buf, s_tx_len)) {
        s_busy = false;
    }
}

bool uvc_streaming(void) { return s_committed && tud_mounted() && tud_video_n_streaming(0, 0); }
int  uvc_format(void)    { return s_fmt; }
void uvc_size(int *w, int *h) { *w = 1920; *h = 1080; }      // siempre la resolución máxima
bool uvc_ready(void)     { return uvc_streaming() && !s_busy; }
uint32_t uvc_frames_sent(void) { return s_frames; }

bool uvc_send(const uint8_t *buf, size_t len)
{
    if (!uvc_ready()) {
        return false;
    }
    s_busy = true;
    s_tx_buf = buf;
    s_tx_len = len;
    s_tx_t0 = esp_timer_get_time();
    usbd_defer_func(do_uvc_xfer, NULL, false);   // se ejecuta en la tarea de TinyUSB
    return true;
}

// ============================ Red NCM <-> lwIP ============================
typedef struct {
    esp_netif_driver_base_t base;
} ncm_driver_t;

static esp_netif_t *s_netif;
static ncm_driver_t s_drv;

static esp_err_t netif_transmit(void *h, void *buffer, size_t len)
{
    if (!tud_mounted()) {
        return ESP_FAIL;
    }
    esp_err_t r = tinyusb_net_send_sync(buffer, len, NULL, pdMS_TO_TICKS(100));
    if (r == ESP_OK) {
        stats_net(0, len);
    }
    return r;
}

static void netif_free_rx(void *h, void *buffer)
{
    free(buffer);
}

static esp_err_t ncm_post_attach(esp_netif_t *esp_netif, void *args)
{
    ncm_driver_t *drv = args;
    drv->base.netif = esp_netif;
    const esp_netif_driver_ifconfig_t ifcfg = {
        .handle = drv,
        .transmit = netif_transmit,
        .driver_free_rx_buffer = netif_free_rx,
    };
    return esp_netif_set_driver_config(esp_netif, &ifcfg);
}

static esp_err_t usb_recv_cb(void *buffer, uint16_t len, void *ctx)
{
    if (!s_netif) {
        return ESP_OK;
    }
    void *copy = malloc(len);
    if (!copy) {
        return ESP_ERR_NO_MEM;
    }
    memcpy(copy, buffer, len);
    stats_net(len, 0);
    return esp_netif_receive(s_netif, copy, len, copy);
}

static void netif_setup(const uint8_t *dev_mac)
{
    esp_netif_ip_info_t ip = {
        .ip.addr = ESP_IP4TOADDR(192, 168, 7, 1),
        .netmask.addr = ESP_IP4TOADDR(255, 255, 255, 0),
        .gw.addr = ESP_IP4TOADDR(192, 168, 7, 1),
    };
    esp_netif_inherent_config_t base = ESP_NETIF_INHERENT_DEFAULT_ETH();
    base.flags = ESP_NETIF_DHCP_SERVER | ESP_NETIF_FLAG_AUTOUP;
    base.ip_info = &ip;
    base.if_key = "USB_NCM";
    base.if_desc = "usb ncm";
    base.route_prio = 10;
    base.get_ip_event = 0;
    base.lost_ip_event = 0;
    esp_netif_config_t cfg = { .base = &base, .driver = NULL, .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH };
    s_netif = esp_netif_new(&cfg);
    s_drv.base.post_attach = ncm_post_attach;
    ESP_ERROR_CHECK(esp_netif_attach(s_netif, &s_drv));
    ESP_ERROR_CHECK(esp_netif_set_mac(s_netif, (uint8_t *)dev_mac));

    // DHCP: el PC recibe 192.168.7.2 SIN puerta de enlace ni DNS, así su internet no cambia.
    uint8_t no = 0;
    esp_netif_dhcps_option(s_netif, ESP_NETIF_OP_SET, ESP_NETIF_ROUTER_SOLICITATION_ADDRESS, &no, sizeof(no));
    esp_netif_dhcps_option(s_netif, ESP_NETIF_OP_SET, ESP_NETIF_DOMAIN_NAME_SERVER, &no, sizeof(no));
    dhcps_lease_t lease = { .enable = true };
    lease.start_ip.addr = ESP_IP4TOADDR(192, 168, 7, 2);
    lease.end_ip.addr = ESP_IP4TOADDR(192, 168, 7, 3);
    esp_netif_dhcps_option(s_netif, ESP_NETIF_OP_SET, ESP_NETIF_REQUESTED_IP_ADDRESS, &lease, sizeof(lease));

    esp_netif_action_start(s_netif, NULL, 0, NULL);
    esp_netif_action_connected(s_netif, NULL, 0, NULL);
}

static volatile bool s_mounted;
bool usb_mounted(void) { return s_mounted; }

static void usb_event(tinyusb_event_t *ev, void *arg)
{
    if (ev->id == TINYUSB_EVENT_ATTACHED) {
        s_mounted = true;
        tud_network_link_state(0, true);
        ESP_LOGI(TAG, "USB conectado al PC (High-Speed: %s)", tud_speed_get() == TUSB_SPEED_HIGH ? "sí" : "no");
    } else if (ev->id == TINYUSB_EVENT_DETACHED) {
        s_mounted = false;
        s_committed = false;
        s_busy = false;
        ESP_LOGW(TAG, "USB desconectado");
    }
}

void usb_start(void)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BASE);
    snprintf(s_serial, sizeof(s_serial), "P4-%02X%02X%02X", mac[3], mac[4], mac[5]);

    uint8_t host_mac[6], dev_mac[6];
    memcpy(host_mac, mac, 6);
    memcpy(dev_mac, mac, 6);
    host_mac[0] = 0x02;        // administrada localmente (la usa el PC)
    dev_mac[0] = 0x06;         // la usa el P4
    dev_mac[5] ^= 0x01;

    tinyusb_config_t cfg = TINYUSB_DEFAULT_CONFIG(usb_event);
    cfg.descriptor.device = &s_dev;
    cfg.descriptor.qualifier = &s_qual;
    cfg.descriptor.string = s_str;
    cfg.descriptor.string_count = STR_COUNT;
    cfg.descriptor.full_speed_config = s_cfg_fs;
    cfg.descriptor.high_speed_config = s_cfg_hs;
    cfg.task.size = 8192;
    cfg.task.priority = 20;
    cfg.task.xCoreID = 0;
    ESP_ERROR_CHECK(tinyusb_driver_install(&cfg));

    tinyusb_net_config_t net = {
        .on_recv_callback = usb_recv_cb,
        .user_context = NULL,
    };
    memcpy(net.mac_addr, host_mac, 6);
    ESP_ERROR_CHECK(tinyusb_net_init(&net));

    netif_setup(dev_mac);
    ESP_LOGI(TAG, "USB listo: webcam UVC (MJPEG/H.264) + red NCM en " P4_IP_STR);
}
