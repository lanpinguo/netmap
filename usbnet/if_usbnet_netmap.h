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



/* board specific private data structure */
struct usbnet_adapter {

	u32 bd_number;
	u32 rx_buffer_len;
	u16 mng_vlan_id;
	u16 link_speed;
	u16 link_duplex;
	u16 eeprom_vers;

	/* track device up/down/testing state */
	unsigned long state;

	/* Interrupt Throttle Rate */
	u32 itr;
	u32 itr_setting;
	u16 tx_itr;
	u16 rx_itr;
	void* rx_buff;
	/* Tx - one ring per active queue */
	struct usbnet_ring *tx_ring ____cacheline_aligned_in_smp;
	u32 tx_fifo_limit;


	unsigned int uncorr_errors;	/* uncorrectable ECC errors */
	unsigned int corr_errors;	/* correctable ECC errors */
	unsigned int restart_queue;
	u32 txd_cmd;

	bool detect_tx_hung;
	bool tx_hang_recheck;
	u8 tx_timeout_factor;

	u32 tx_int_delay;
	u32 tx_abs_int_delay;

	unsigned int total_tx_bytes;
	unsigned int total_tx_packets;
	unsigned int total_rx_bytes;
	unsigned int total_rx_packets;

	/* Tx stats */
	u64 tpt_old;
	u64 colc_old;
	u32 gotc;
	u64 gotc_old;
	u32 tx_timeout_count;
	u32 tx_fifo_head;
	u32 tx_head_addr;
	u32 tx_fifo_size;
	u32 tx_dma_failed;
	u32 tx_hwtstamp_timeouts;
	u32 tx_hwtstamp_skipped;

	/* Rx */
	bool (*clean_rx)(struct usbnet_ring *ring) ____cacheline_aligned_in_smp;
	void (*alloc_rx_buf)(struct usbnet_ring *ring, int cleaned_count,
			     gfp_t gfp);
	struct usbnet_ring *rx_ring;

	u32 rx_int_delay;
	u32 rx_abs_int_delay;

	/* Rx stats */
	u64 hw_csum_err;
	u64 hw_csum_good;
	u64 rx_hdr_split;
	u32 gorc;
	u64 gorc_old;
	u32 alloc_rx_buff_failed;
	u32 rx_dma_failed;
#ifdef HAVE_HW_TIME_STAMP
	u32 rx_hwtstamp_cleared;
#endif

	unsigned int rx_ps_pages;
	u16 rx_ps_bsize0;
#ifndef CONFIG_E1000E_NAPI
	u64 rx_dropped_backlog;		/* count drops from rx int handler */
#endif
	u32 max_frame_size;
	u32 min_frame_size;

	/* OS defined structs */
	struct net_device *netdev;

#ifndef HAVE_NETDEV_STATS_IN_NETDEV
	struct net_device_stats net_stats;
#endif

	u32 test_icr;

	u32 msg_enable;
	unsigned int num_vectors;
	int int_mode;
	u32 eiac_mask;

	u32 eeprom_wol;
	u32 wol;
	u32 pba;
	u32 max_hw_frame_size;

	bool fc_autoneg;

#ifndef HAVE_ETHTOOL_SET_PHYS_ID
	unsigned long led_status;

#endif
	unsigned int flags;
	unsigned int flags2;
	struct work_struct downshift_task;
	struct work_struct update_phy_task;
	struct work_struct print_hang_task;
	u32 *config_space;

	int node; /* store the node to allocate memory on */
	int phy_hang_count;

	u16 tx_ring_count;
	u16 rx_ring_count;
	u8 revision_id;


	s32 ptp_delta;
	u16 eee_advert;
};



#define SOFTC_T	usbnet_adapter

#define usbnet_driver_name netmap_usbnet_driver_name
char netmap_usbnet_driver_name[] = "usbnet" NETMAP_LINUX_DRIVER_SUFFIX;

/*
 * Adaptation to different versions of the driver.
 */
#ifdef NETMAP_LINUX_HAVE_USBNET_EXT_RXDESC
#warning this driver uses extended descriptors
#define NM_E1K_RX_DESC_T	union usbnet_rx_desc_extended
#define	NM_E1R_RX_STATUS	wb.upper.status_error
#define	NM_E1R_RX_LENGTH	wb.upper.length
#define	NM_E1R_RX_BUFADDR	read.buffer_addr
#else
#warning this driver uses regular descriptors
#define USBNET_RX_DESC_EXT	USBNET_RX_DESC	// XXX workaround
#define NM_E1K_RX_DESC_T	struct usbnet_rx_desc
#define	NM_E1R_RX_STATUS	status
#define	NM_E1R_RX_BUFADDR	buffer_addr
#define	NM_E1R_RX_LENGTH	length
#endif /* up to 3.2.x */

#ifndef NETMAP_LINUX_HAVE_USBNET_HWADDR
#define NM_WR_TX_TAIL(_x)	writel(_x, txr->tail)	// XXX tx_ring
#define	NM_WR_RX_TAIL(_x)	writel(_x, rxr->tail)	// XXX rx_ring
#define	NM_RD_TX_HEAD()		readl(txr->head)
#else
#define NM_WR_TX_TAIL(_x)	writel(_x, adapter->hw.hw_addr + txr->tail)
#define	NM_WR_RX_TAIL(_x)	writel(_x, adapter->hw.hw_addr + rxr->tail)
#define	NM_RD_TX_HEAD()		readl(adapter->hw.hw_addr + txr->head)
#endif


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



static void dummy(void){}

#ifdef NETMAP_LINUX_HAVE_USBNET_DOWN2
#define nm_usbnet_down(_a)	dummy()
#else
#define nm_usbnet_down(_a)	dummy()
#endif


/*
 * Reconcile kernel and user view of the transmit ring.
 */
static int
usbnet_netmap_txsync(struct netmap_kring *kring, int flags)
{
	D("usbnet_netmap_txsync  not be supported");
	return 0;
}


/*
 * Reconcile kernel and user view of the receive ring.
 */
static int
usbnet_netmap_rxsync(struct netmap_kring *kring, int flags)
{
	struct netmap_adapter *na = kring->na;
	struct ifnet *ifp = na->ifp;
	struct netmap_ring *ring = kring->ring;
	u_int ring_nr = kring->ring_id;
	u_int nm_i;		/* index into the netmap ring */
	u_int nic_i = 1;	/* index into the NIC ring */
	u_int n;
	u_int const lim = kring->nkr_num_slots - 1;
	u_int const head = kring->rhead;
	int force_update = (flags & NAF_FORCE_READ) || kring->nr_kflags & NKR_PENDINTR;

	if (!netif_carrier_ok(ifp))
		return 0;

	if (head > lim)
		return netmap_ring_reinit(kring);

	rmb();

	nm_i = netmap_idx_n2k(kring, nic_i);


	kring->nr_hwtail = nm_i;
	kring->nr_kflags &= ~NKR_PENDINTR;

	
	return 0;
}


/* diagnostic routine to catch errors */
static void usbnet_no_rx_alloc(struct usbnet *a, int n)
{
	D("usbnet->alloc_rx_buf should not be called");
}




/*
 * Make the tx and rx rings point to the netmap buffers.
 */
static void* usbnet_netmap_buffer_get(struct net_device *dev)
{
	struct usbnet_adapter 	*ua;
	ua = usbnet_adapter(dev);

	if(ua)
	{
		return ua->rx_buff;
	}

	return NULL;
}


/*
 * Make the tx and rx rings point to the netmap buffers.
 */
static int usbnet_netmap_init_buffers(struct net_device *dev)
{
	struct usbnet_adapter 	*ua;
	struct usbnet			*ud;
	struct ifnet 			*ifp ;
	struct netmap_adapter	*na ;
	struct netmap_slot		*slot;
	int i, si;
	uint64_t vaddr;

	D("usbnet_netmap_init_buffers enter");

	ud = netdev_priv(dev);
	ua = usbnet_adapter(dev);

	if(ua == NULL)
	{
		D("usbnet_netmap_init_buffers ua is null");
		return -1;
	}

	ifp = ud->net;
	na = NA(ifp);
	
	ua->bd_number 		= 0x1011;
	ua->rx_buffer_len 	= 1024 ;
	ua->mng_vlan_id		= 4094;
	ua->link_speed		= 1000;
	ua->link_duplex		= 0x1;
	ua->eeprom_vers		= 0x1015;

#if 1
	if (!nm_native_on(na))
		return 0;
#endif
	slot = netmap_reset(na, NR_RX, 0, 0);
	if (slot) {
		/* initialize the RX ring for netmap mode */
		D("rx slot %x was set", slot);
		for (i = 0; i < 1; i++) {
			si = netmap_idx_n2k(&na->rx_rings[0], i);
			D("rx si %x was set", si);
			//vaddr = NMB(na, slot + si);
			D("rx vaddr %x was set", (slot + si)->ptr);
			ua->rx_buff = (slot + si)->ptr;
		}
		
		D("rx slot rx_buff 0x%0x was get", ua->rx_buff);
	}

	slot = netmap_reset(na, NR_TX, 0, 0);
	if (slot) {
		/* initialize the tx ring for netmap mode */
		D("tx slot %d was set", slot);
	}
	return 1;



	D("usbnet_netmap_init_buffers not be supported");
	return 0;
}




/*
 * Register/unregister. We are already under netmap lock.
 */
static int
usbnet_netmap_reg(struct netmap_adapter *na, int onoff)
{
	struct ifnet *ifp = na->ifp;
	struct usbnet *adapter = netdev_priv(ifp);

	/* protect against other reinit */
#if 0
	while (test_and_set_bit(__USBNET_RESETTING, &adapter->state))
		usleep_range(1000, 2000);
#endif

	if (netif_running(adapter->net))
		nm_usbnet_down(adapter);

	/* enable or disable flags and callbacks in na and ifp */
	if (onoff) {
		nm_set_native_flags(na);
		usbnet_netmap_init_buffers(ifp);
	} else {
		nm_clear_native_flags(na);
	}

#if 0
	if (netif_running(adapter->netdev))
		usbnet_up(adapter);
	else
		usbnet_reset(adapter);	// XXX is it needed ?

	clear_bit(__USBNET_RESETTING, &adapter->state);
#endif
	return (0);
}



static void
usbnet_netmap_attach(	struct net_device *dev)
{
	struct netmap_adapter 	na;
	struct usbnet_adapter 	*ua;
	struct usbnet			*ud;

	bzero(&na, sizeof(na));

	ud = netdev_priv(dev);
	ua = usbnet_adapter(dev);
	
	na.ifp = ud->net;
	na.pdev = NULL;
	na.num_tx_desc = 5;
	na.num_rx_desc = 5;
	na.nm_register = usbnet_netmap_reg;
	na.nm_txsync = usbnet_netmap_txsync;
	na.nm_rxsync = usbnet_netmap_rxsync;
	na.num_tx_rings = na.num_rx_rings = 1;
	netmap_attach(&na);
}

/* end of file */
