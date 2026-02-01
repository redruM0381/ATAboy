#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#define CFG_TUSB_RHPORT0_MODE    OPT_MODE_DEVICE
#define CFG_TUD_CDC              1  // <--- This enables CDC
#define CFG_TUD_MSC              1  // <--- This enables Mass Storage
#define CFG_TUD_MSC_EP_BUFSIZE   4096

#define CFG_TUD_CDC_RX_BUFSIZE   64
#define CFG_TUD_CDC_TX_BUFSIZE   64

#endif
