/*
 * Copyright (C) 2012-2014 Gaetano Catalli, Luigi Rizzo. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * $Id: if_usbnet_netmap.h 10670 2017-12-30 21:15:38Z lanpinguo $
 *
 * netmap support for: usbnet (linux version)
 * For details on netmap support please see ixgbe_netmap.h
 * The driver supports 1 TX and 1 RX ring. Single lock.
 * tx buffer address only written on change.
 * Apparently the driver uses extended descriptors on rx from 3.2.32
 * Rx Crc stripping ?
 */


#include <bsd_glue.h>
#include <net/netmap.h>
#include <netmap/netmap_kern.h>



/* wrappers around a pointer to a socket buffer,
 * so a DMA handle can be stored along with the buffer
 */
struct usbnet_buffer {
	//dma_addr_t dma;
	struct sk_buff *skb;
	union {
		/* Tx */
		struct {
			unsigned long time_stamp;
			u16 length;
			u16 next_to_watch;
			unsigned int segs;
			unsigned int bytecount;
			u16 mapped_as_page;
		};
		/* Rx */
		struct {
			/* arrays of page information for packet split */
			//struct usbnet_ps_page *ps_pages;
			struct page *page;
		};
	};
};


struct usbnet_ring {
	struct usbnet_adapter *adapter;	/* back pointer to adapter */
	void *desc;			/* pointer to ring memory  */
	//dma_addr_t dma;			/* phys address of ring    */
	unsigned int size;		/* length of ring in bytes */
	unsigned int count;		/* number of desc. in ring */

	u16 next_to_use;
	u16 next_to_clean;

	void __iomem *head;
	void __iomem *tail;

	/* array of buffer information structs */
	struct usbnet_buffer *buffer_info;

	char name[IFNAMSIZ + 5];
	u32 ims_val;
	u32 itr_val;
	void __iomem *itr_register;
	int set_itr;

	struct sk_buff *rx_skb_top;
};

#define MAX_NIC_RX_RING	10

struct usbnet_slot{

	void*	data_ptr;
	u64		data_len;
	u32		in_use;

#define SLOT_STATE_IDLE 0	
#define SLOT_STATE_USED	1	

};


/* board specific private data structure */
struct usbnet_adapter {
	u32 				num_rx_buff;
	struct usbnet_slot  rx_ring[MAX_NIC_RX_RING];
	u32					rdh; /*Rx Ring head index*/
	u32					rdt; /*Rx Ring tail index*/
	u32					cur; /*Rx Ring current nic index for read*/
	u64 				rx_pkts_cnt;
	u64					rx_drop_pkts_cnt;

	u32 				num_tx_buff;
	u32					tdh; /*Tx Ring head index*/
	u32					tdt; /*Tx Ring tail index*/
	u64 				tx_pkts_cnt;
	u64					tx_drop_pkts_cnt;

};

#define RESERVED_BLOCK_SIZE		256
#define KDP_GET_FEILD_ADDR(pcb,feild)	((void*)(pcb) + pcb->feilds[feild].offset)

enum{
	FEILD_DMAC = 0,	/* destination mac address*/
	FEILD_SMAC,			/* source mac address */
	FEILD_VLAN_0,
	FEILD_VLAN_1,
	FEILD_L3_TYPE,	/* layer 3 protocol type */
	FEILD_MPLS_2, 	/* mpls2 header */
	FEILD_MPLS_1,		/* mpls1 header */
	FEILD_MPLS_0, 	/* mpls0 header */
	FEILD_CW, 			/* mpls control word */
	FEILD_L3_HDR, 	/* layer 3 protocol header  */
	FEILD_L4_HDR,		/* layer 4 protocol header */
	FEILD_DATA, 		/* payload */
	
	FEILD_MAX

};

enum{
	FEILD_DMAC_LEN 		= 6,	/* destination mac address*/
	FEILD_SMAC_LEN 		= 6,			/* source mac address */
	FEILD_VLAN_0_LEN	= 4,
	FEILD_VLAN_1_LEN	= 4,
	FEILD_L3_TYPE_LEN	= 2,	/* layer 3 protocol type */
	FEILD_MPLS_2_LEN	= 4,		/* mpls2 header */
	FEILD_MPLS_1_LEN	= 4,		/* mpls1 header */
	FEILD_MPLS_0_LEN	= 4,		/* mpls0 header */
	FEILD_CW_LEN			= 4, 			/* mpls control word */
	FEILD_L3_HDR_LEN	= 20, 	/* layer 3 protocol header	*/
	FEILD_L4_HDR_LEN	= 20, 	/* layer 4 protocol header */

};


typedef struct kofdpa_pkt_feild_s {
	uint16_t									offset; /* the feild location offset from the base address */
	uint16_t									len; 		/* the feild length */
}kofdpa_pkt_feild_t;

typedef struct kofdpa_MetaData_s
{
  uint32_t    mplsL2Port;               /**< For MPLS L2 VPN classification. */
  uint32_t    tunnelId;                 /**< For MPLS L2 VPN classification. */
  uint32_t    mplsType;               /**< MPLS Type value used in Set-field action */
	void				*pGrpInst;
}kofdpa_MetaData_t;

typedef struct kofdpaPktCb_s
{
	uint64_t								port; 	/* every bit indicate a port, physical port number range is 0~63 */
	void 										*this;	/* pointer to self*/
	uint32_t								len;		/* total len */
	uint16_t								pkt_len;/* pkt len */
	uint16_t								cur;		/* point current position in the packet */
	uint16_t								pool_tail;		/* the tail postion of the memory pool for packet increase */
	uint16_t								pool_head;		/* the head postion of the memory pool for packet increase */
	kofdpa_pkt_feild_t 			feilds[FEILD_MAX];		/* packet feild info */
	kofdpa_MetaData_t				meta_data;
	void* 									action_set[2];
}kofdpaPktCb_t;





#define SOFTC_T	usbnet_adapter

#define usbnet_driver_name netmap_usbnet_driver_name

/*
 * Adaptation to different versions of the driver.
 */
#define USBNET_RX_DESC_EXT	USBNET_RX_DESC	// XXX workaround
#define NM_E1K_RX_DESC_T	struct usbnet_rx_desc
#define	NM_E1R_RX_STATUS	status
#define	NM_E1R_RX_BUFADDR	buffer_addr
#define	NM_E1R_RX_LENGTH	length


#define NM_WR_TX_TAIL(_x)	writel(_x, ua->tdt)	// XXX tx_ring
#define	NM_WR_RX_TAIL(_x)	writel(_x, ua->rdt)	// XXX rx_ring
#define	NM_RD_TX_HEAD()		ua->tdt

/**
 *	netdev_priv - access network device private data
 *	@dev: network device
 *
 * Get network device private data
 */
static inline void *usbnet_adapter(const struct net_device *dev)
{
	return (char *)dev + ALIGN(sizeof(struct net_device), NETDEV_ALIGN)
		+ ALIGN(sizeof(struct usbnet), NETDEV_ALIGN) ;
}






int usbnet_netmap_txsync(struct netmap_kring *kring, int flags);

int usbnet_netmap_rxsync(struct netmap_kring *kring, int flags);

void usbnet_no_rx_alloc(struct usbnet *a, int n);

void* usbnet_netmap_buffer_get(struct usbnet_adapter 	*ua);

int usbnet_netmap_init_buffers(struct net_device *dev);

int usbnet_netmap_reg(struct netmap_adapter *na, int onoff);

void usbnet_netmap_attach(	struct net_device *dev);

int usbnet_netmap_rx_fixup(struct net_device *dev, struct sk_buff * skb);

/* end of file */
