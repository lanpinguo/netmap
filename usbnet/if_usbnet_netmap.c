/*
 * USB Network driver infrastructure
 * Copyright (C) 2000-2005 by David Brownell
 * Copyright (C) 2003-2005 David Hollis <dhollis@davehollis.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/*
 * This is a generic "USB networking" framework that works with several
 * kinds of full and high speed networking devices:  host-to-host cables,
 * smart usb peripherals, and actual Ethernet adapters.
 *
 * These devices usually differ in terms of control protocols (if they
 * even have one!) and sometimes they define new framing to wrap or batch
 * Ethernet packets.  Otherwise, they talk to USB pretty much the same,
 * so interface (un)binding, endpoint I/O queues, fault handling, and other
 * issues can usefully be addressed by this framework.
 */

// #define	DEBUG			// error path messages, extra info
// #define	VERBOSE			// more; success messages

#include <linux/module.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ctype.h>
#include <linux/ethtool.h>
#include <linux/workqueue.h>
#include <linux/mii.h>
#include <linux/usb.h>
#include <linux/usb/usbnet.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/pm_runtime.h>


#include <bsd_glue.h>
#include <net/netmap.h>
#include <netmap/netmap_kern.h>
#include "if_usbnet_netmap.h"


char netmap_usbnet_driver_name[] = "usbnet" NETMAP_LINUX_DRIVER_SUFFIX;



/*
 * Reconcile kernel and user view of the transmit ring.
 */
int
usbnet_netmap_txsync(struct netmap_kring *kring, int flags)
{
	struct netmap_adapter *na = kring->na;
	struct ifnet *ifp = na->ifp;
	struct netmap_ring *ring = kring->ring;
	u_int ring_nr = kring->ring_id;
	u_int nm_i; /* index into the netmap ring */
	u_int nic_i;	/* index into the NIC ring */
	u_int n;
	u_int const lim = kring->nkr_num_slots - 1;
	u_int const head = kring->rhead;
	/* generate an interrupt approximately every half ring */
	u_int report_frequency = kring->nkr_num_slots >> 1;

	/* device-specific */
	struct usbnet_adapter 	*ua;


	ua = usbnet_adapter(ifp);


	rmb();
	/*
	 * First part: process new packets to send.
	 */

	if (!netif_carrier_ok(ifp)) {
		goto out;
	}

	nm_i = kring->nr_hwcur;
	if (nm_i != head) { /* we have new packets to send */
		nic_i = netmap_idx_k2n(kring, nm_i);
		for (n = 0; nm_i != head; n++) {
			struct netmap_slot *slot = &ring->slot[nm_i];
			u_int len = slot->len;
			uint64_t paddr;
			void *addr = PNMB(na, slot, &paddr);

			/* device-specific */
			/*Todo*/

			
			NM_CHECK_ADDR_LEN(na, addr, len);

			if (slot->flags & NS_BUF_CHANGED) {
				/* buffer has changed, reload map */
				// netmap_reload_map(pdev, DMA_TO_DEVICE, old_paddr, addr)
				//curr->buffer_addr = htole64(paddr);
			}
			slot->flags &= ~(NS_REPORT | NS_BUF_CHANGED);

			/* Fill the slot in the NIC ring. */
			/*Todo*/	
		
			nm_i = nm_next(nm_i, lim);
			nic_i = nm_next(nic_i, lim);
		}
		kring->nr_hwcur = head;

		wmb();	/* synchronize writes to the NIC ring */

	}

	/*
	 * Second part: reclaim buffers for completed transmissions.
	 */
	if (flags & NAF_FORCE_RECLAIM || nm_kr_txempty(kring)) {
		/* record completed transmissions using TDH */
		nic_i = NM_RD_TX_HEAD();	// XXX could scan descriptors ?
		if (nic_i >= kring->nkr_num_slots) { /* XXX can it happen ? */
			D("TDH wrap %d", nic_i);
			nic_i -= kring->nkr_num_slots;
		}
		kring->nr_hwtail = nm_prev(netmap_idx_n2k(kring, nic_i), lim);
	}
out:

	return 0;
}


/*
 * Reconcile kernel and user view of the receive ring.
 */
int
usbnet_netmap_rxsync(struct netmap_kring *kring, int flags)
{
	struct netmap_adapter *na = kring->na;
	struct ifnet *ifp = na->ifp;
	struct netmap_ring *ring = kring->ring;
	u_int ring_nr = kring->ring_id;
	u_int nm_i;		/* index into the netmap ring */
	u_int nic_i ;	/* index into the NIC ring */
	u_int n;
	u_int const lim = kring->nkr_num_slots - 1;
	u_int const head = kring->rhead;
	int force_update = (flags & NAF_FORCE_READ) || kring->nr_kflags & NKR_PENDINTR;
	struct usbnet_adapter 	*ua;


	ua = usbnet_adapter(ifp);

	if (!netif_carrier_ok(ifp))
		return 0;

	if (head > lim)
		return netmap_ring_reinit(kring);

	rmb();


	/*
	 * First part: import newly received packets.
	 */
	if (netmap_no_pendintr || force_update) {
		uint16_t slot_flags = kring->nkr_slot_flags;
	
		nic_i = ua->cur;
		nm_i = netmap_idx_n2k(kring, nic_i);
	
		for (n = 0; ; n++) {
			struct usbnet_slot *curr = &ua->rx_ring[nic_i];
	
			if (curr->in_use == 0)
				break;
			ring->slot[nm_i].len = curr->data_len;
			ring->slot[nm_i].flags = slot_flags;
			nm_i = nm_next(nm_i, lim);
			nic_i = nm_next(nic_i, lim);
		}
		if (n) { /* update the state variables */
			ua->cur = nic_i;
			kring->nr_hwtail = nm_i;
		}
		kring->nr_kflags &= ~NKR_PENDINTR;
	}

	D("rx_pkts =  %lld , drop_pkts = %lld ,cur_i = %lld",ua->rx_pkts_cnt,ua->rx_drop_pkts_cnt,ua->cur);
	/*
	 * Second part: skip past packets that userspace has released.
	 */
	nm_i = kring->nr_hwcur;
	if (nm_i != head) {
		nic_i = netmap_idx_k2n(kring, nm_i);
		for (n = 0; nm_i != head; n++) {
			struct netmap_slot *slot = &ring->slot[nm_i];
			struct usbnet_slot *curr = &ua->rx_ring[nic_i];

			if (slot->flags & NS_BUF_CHANGED) {
				/* buffer has changed, reload map */
				// netmap_reload_map(pdev, DMA_TO_DEVICE, old_paddr, addr)
				slot->flags &= ~NS_BUF_CHANGED;
			}
			curr->in_use = 0;
			nm_i = nm_next(nm_i, lim);
			nic_i = nm_next(nic_i, lim);
		}
		kring->nr_hwcur = head;
		wmb();

	}


	return 0;

ring_reset:
	return netmap_ring_reinit(kring);
}


/* diagnostic routine to catch errors */
void usbnet_no_rx_alloc(struct usbnet *a, int n)
{
	D("usbnet->alloc_rx_buf should not be called");
}




/*
 * Make the tx and rx rings point to the netmap buffers.
 */
void* usbnet_netmap_buffer_get(struct usbnet_adapter 	*ua)
{
#if 0
	void * pBuff;
	if(ua->rdh == ua->rdt)
	{
		D("rx ring buff is full , overrided ");
		
		return NULL;
	}

	pBuff = ua->rx_ring_skb[ua->rdh];
	D("cur rx ring skb buff[%d] = %p ",ua->rdh,ua->rx_ring_skb[ua->rdh]);
	ua->rdh = (ua->rdh + 1) % ua->num_rx_buff;


	return pBuff;
#else
	return NULL;
#endif
}


int usbnet_nic_ring_head_forward(struct usbnet_adapter 	*ua)
{

	ua->rdh = (++ua->rdh) % ua->num_rx_buff;
	return ua->rdh;
}

int usbnet_nic_ring_tail_forward(struct usbnet_adapter 	*ua)
{
	ua->rdt = (++ua->rdt) % ua->num_rx_buff;
	return ua->rdt;
}


/*
 * Make the tx and rx rings point to the netmap buffers.
 */
int usbnet_netmap_rx_fixup(struct net_device *dev, struct sk_buff * skb)
{
	u32 i;
	struct usbnet_adapter 	*ua;
	struct netmap_adapter	*na ;
	struct usbnet_slot		*pRxBuff = NULL;


	ua = usbnet_adapter(dev);
	na = NA(dev);


	/*for(i = 0 ; i < 32 ; i++){
		printk(KERN_WARNING "%02x ",skb->data[i]);
	}*/


	pRxBuff = &ua->rx_ring[ua->rdh];
	
	if (pRxBuff){
		
		if(pRxBuff->in_use != 0){
			D("no rx buffers");
			ua->rx_drop_pkts_cnt++;
			return -1;
		}
		
		memcpy(pRxBuff->data_ptr,skb->data,skb->len);
		pRxBuff->data_len = skb->len;
		pRxBuff->in_use = 1;
		ua->rx_pkts_cnt++;
	}

	/*Update rdh pointer*/
	usbnet_nic_ring_head_forward(ua);
	
	return 0;
	
}
EXPORT_SYMBOL_GPL(usbnet_netmap_rx_fixup);



/*
 * Make the tx and rx rings point to the netmap buffers.
 */
int usbnet_netmap_init_buffers(struct net_device *dev)
{
	struct usbnet_adapter 	*ua;
	struct usbnet			*ud;
	struct ifnet 			*ifp ;
	struct netmap_adapter	*na ;
	struct netmap_slot		*slot;
	int i, si;
	void* vaddr;


	D("usbnet_netmap_init_buffers enter");

	ud = netdev_priv(dev);
	ua = usbnet_adapter(dev);

	if(ua == NULL)
	{
		D("usbnet_netmap_init_buffers ua is null");
		return -1;
	}

	ifp = ud->net;
	na = NA(dev);
	ND("na %p lut %p plut %p bufs %u size %u", na, na->na_lut.lut,na->na_lut.plut, na->na_lut.objtotal,
				    na->na_lut.objsize);	
	ua->num_rx_buff		= 5;

#if 1
	if (!nm_native_on(na))
		return 0;
#endif
	slot = netmap_reset(na, NR_RX, 0, 0);
	if (slot) {
		/* initialize the RX ring for netmap mode */
		D("rx slot %llx was set", slot);
		for (i = 0; i < ua->num_rx_buff; i++) {
			si = netmap_idx_n2k(&na->rx_rings[0], i);
			vaddr = NMB(na, slot + si);
			ua->rx_ring[i].data_ptr = vaddr;
			ua->rx_ring[i].in_use = 0;
			//*((u32*)vaddr + 8) = 0x11111111 * (i + 1);
			//ua->rx_ring_skb[i] = build_skb(vaddr, 1500);

			
		}
		ua->cur = 0;
		ua->rdh = 0;
		ua->rdt = ua->num_rx_buff - 1;
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
int
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
	if (netif_running(adapter->net))
		usbnet_up(adapter);
	else
		usbnet_reset(adapter);	// XXX is it needed ?

	clear_bit(__USBNET_RESETTING, &adapter->state);
#endif	
	return (0);
}



void
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




