// SPDX-License-Identifier: GPL-2.0

/* CAN bus driver for Microchip 25XXFD CAN Controller with SPI Interface
 * implementation of can transmission
 *
 * Copyright 2017 Martin Sperl <kernel@martin.sperl.org>
 *
 * Based on Microchip MCP251x CAN controller driver written by
 * David Vrabel, Copyright 2006 Arcom Control Systems Ltd.
 */

#include "mcp25xxfd_can.h"
#include <linux/can/core.h>
#include <linux/can/dev.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/netdevice.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>

#include <linux/module.h>
bool debug;
module_param(debug, bool, 0664);
MODULE_PARM_DESC(debug, "enable debug");

/* structure of a spi message that is prepared and can get deployed quickly */
struct mcp2517fd_tx_spi_message {
	/* the list where this object belongs to - idle or queued */
	struct list_head list;
	/* the network device this is related to */
	struct net_device *net;
	/* the spi_message to get submitted async */
	struct spi_message msg;
	/* the fifo this fills */
	u32 fifo;
	/* the xfer to fill in the fifo data */
	struct {
		struct spi_transfer xfer;
		struct {
			u8 cmd[2];
			u8 header[sizeof(struct mcp25xxfd_obj_tx)];
			u8 data[64];
		} data;
	} fill_fifo;
	/* the xfer to enable transmission on the can bus */
	struct {
		struct spi_transfer xfer;
		struct {
			u8 cmd[2];
			u8 data;
		} data;
	} trigger_fifo;
};

struct mcp2517fd_tx_spi_message_queue {
	/* spinlock protecting the following members */
	spinlock_t lock;
	struct list_head idle;
	struct list_head queued;
	int state;
	/* spinlock protectinb pending_fifos */
	spinlock_t pending_lock;
	u32 pending_fifos;
	struct mcp2517fd_tx_spi_message queue[];
};

static void mcp25xxfd_can_tx_spi_message_complete(void *context)
{
	struct mcp2517fd_tx_spi_message *msg = context;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(msg->net);
	unsigned long flags;

	/* reset transfer length to without data (DLC = 0) */
	msg->fill_fifo.xfer.len = sizeof(msg->fill_fifo.data.cmd) +
		sizeof(msg->fill_fifo.data.header);

	/* mark this fifo as pending transmission in pending_fifos bitmask */
	spin_lock_irqsave(&cpriv->fifos.tx_queue->pending_lock, flags);
	cpriv->fifos.tx_queue->pending_fifos |= BIT(msg->fifo);
	spin_unlock_irqrestore(&cpriv->fifos.tx_queue->pending_lock, flags);
}

static
void mcp25xxfd_can_tx_message_init(struct net_device *net,
				   struct mcp2517fd_tx_spi_message *msg,
				   int fifo)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	struct mcp25xxfd_priv *priv = cpriv->priv;
	const u32 trigger = CAN_FIFOCON_TXREQ | CAN_FIFOCON_UINC;
	const int first_byte = mcp25xxfd_first_byte(trigger);
	u32 addr;

	/* and initialize the structure */
	msg->net = net;
	msg->fifo = fifo;
	spi_message_init(&msg->msg);

	/* message callback */
	msg->msg.complete = mcp25xxfd_can_tx_spi_message_complete;
	msg->msg.context = msg;

	/* init fill_fifo */
	msg->fill_fifo.xfer.speed_hz = priv->spi_use_speed_hz;
	msg->fill_fifo.xfer.cs_change = true;
	msg->fill_fifo.xfer.tx_buf = msg->fill_fifo.data.cmd;
	msg->fill_fifo.xfer.len = sizeof(msg->fill_fifo.data.cmd) +
		sizeof(msg->fill_fifo.data.header);
	spi_message_add_tail(&msg->fill_fifo.xfer, &msg->msg);

	/* init trigger_fifo */
	msg->trigger_fifo.xfer.speed_hz = priv->spi_use_speed_hz;
	msg->trigger_fifo.xfer.tx_buf = msg->trigger_fifo.data.cmd;
	msg->trigger_fifo.xfer.len = sizeof(msg->trigger_fifo.data.cmd) +
		sizeof(msg->trigger_fifo.data.data);
	spi_message_add_tail(&msg->trigger_fifo.xfer, &msg->msg);

	/* fill_fifo */
	addr = MCP25XXFD_SRAM_ADDR(cpriv->fifos.fifo_reg[fifo].offset);
	mcp25xxfd_calc_cmd_addr(INSTRUCTION_WRITE,
				addr, msg->fill_fifo.data.cmd);

	/* trigger_fifo */
	mcp25xxfd_calc_cmd_addr(INSTRUCTION_WRITE,
				CAN_FIFOCON(fifo) + first_byte,
				msg->trigger_fifo.data.cmd);
	msg->trigger_fifo.data.data = trigger >> (8 * first_byte);

	/* and add to idle tx transfers */
	list_add_tail(&msg->list, &cpriv->fifos.tx_queue->idle);
}

int mcp25xxfd_can_tx_queue_alloc(struct net_device *net)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	size_t size = sizeof(struct mcp2517fd_tx_spi_message_queue) +
		cpriv->fifos.tx.count *
		sizeof(struct mcp2517fd_tx_spi_message);
	struct mcp2517fd_tx_spi_message *msg;
	int i, fifo;

	/* allocate the fifos as an array */
	cpriv->fifos.tx_queue = kzalloc(size, GFP_KERNEL);
	if (!cpriv->fifos.tx_queue)
		return -ENOMEM;

	/* initialize the tx_queue structure */
	spin_lock_init(&cpriv->fifos.tx_queue->lock);
	spin_lock_init(&cpriv->fifos.tx_queue->pending_lock);
	INIT_LIST_HEAD(&cpriv->fifos.tx_queue->idle);
	INIT_LIST_HEAD(&cpriv->fifos.tx_queue->queued);

	/* initialize the individual spi_message structures */
	for (i = 0, fifo = cpriv->fifos.tx.start;
	     i < cpriv->fifos.tx.count;
	     i++, fifo += cpriv->fifos.tx.increment) {
		msg = &cpriv->fifos.tx_queue->queue[i];
		mcp25xxfd_can_tx_message_init(net, msg, fifo);
	}

	return 0;
}

void mcp25xxfd_can_tx_queue_free(struct net_device *net)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);

	/* eventually we may need to wait here
	 * for all transfers to have finished
	 */

	kfree(cpriv->fifos.tx_queue);
	cpriv->fifos.tx_queue = NULL;
}

void mcp25xxfd_can_tx_queue_manage_nolock(struct net_device *net, int state)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);

	/* skip early */
	if (state == cpriv->fifos.tx_queue->state)
		return;

	/* start/stop netif_queue if necessary */
	switch (cpriv->fifos.tx_queue->state) {
	case TX_QUEUE_STATE_RUNABLE:
		switch (state) {
		case TX_QUEUE_STATE_RESTART:
		case TX_QUEUE_STATE_STARTED:
			netif_start_queue(net);
			cpriv->fifos.tx_queue->state =
				TX_QUEUE_STATE_STARTED;
			break;
		}
		break;
	case TX_QUEUE_STATE_STOPPED:
		switch (state) {
		case TX_QUEUE_STATE_STARTED:
			netif_start_queue(net);
			cpriv->fifos.tx_queue->state = state;
			break;
		}
		break;
	case TX_QUEUE_STATE_STARTED:
		switch (state) {
		case TX_QUEUE_STATE_RUNABLE:
		case TX_QUEUE_STATE_STOPPED:
			netif_stop_queue(net);
			cpriv->fifos.tx_queue->state = state;
			break;
		}
		break;
	default:
		WARN(true, "Unsupported tx_queue state: %i\n",
		     cpriv->fifos.tx_queue->state);
		break;
	}
}

void mcp25xxfd_can_tx_queue_manage(struct net_device *net, int state)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	unsigned long flags;

	spin_lock_irqsave(&cpriv->fifos.tx_queue->lock, flags);

	mcp25xxfd_can_tx_queue_manage_nolock(net, state);

	spin_unlock_irqrestore(&cpriv->fifos.tx_queue->lock, flags);
}

void mcp25xxfd_can_tx_queue_restart(struct net_device *net)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	unsigned long flags;

	spin_lock_irqsave(&cpriv->fifos.tx_queue->pending_lock, flags);

	/* if pending is not empty then we skip */
	if (cpriv->fifos.tx_queue->pending_fifos)
		goto out;

	/* if idle is not empty then skip */
	if (!list_empty(&cpriv->fifos.tx_queue->idle))
		goto out;

	/* move all items from queued to idle */
	list_splice_init(&cpriv->fifos.tx_queue->queued,
			 &cpriv->fifos.tx_queue->idle);
	/* and enable queue */
	mcp25xxfd_can_tx_queue_manage_nolock(net, TX_QUEUE_STATE_RESTART);
out:
	spin_unlock_irqrestore(&cpriv->fifos.tx_queue->pending_lock, flags);
}

static int mcp25xxfd_can_int_handle_txatif_fifo(struct spi_device *spi,
						int fifo)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	u32 val;
	unsigned long flags;
	int ret;

	/* read fifo status */
	ret = mcp25xxfd_cmd_read(spi, CAN_FIFOSTA(fifo), &val);
	if (ret)
		return ret;

	/* clear the relevant interrupt flags */
	ret = mcp25xxfd_cmd_write_mask(spi,
				       CAN_FIFOSTA(fifo),
				       0,
				       CAN_FIFOSTA_TXABT |
				       CAN_FIFOSTA_TXLARB |
				       CAN_FIFOSTA_TXERR |
				       CAN_FIFOSTA_TXATIF);
	if (ret)
		return ret;

	spin_lock_irqsave(&cpriv->fifos.tx_queue->pending_lock, flags);
	/* for specific cases we probably could trigger a retransmit
	 * instead of an abort.
	 */

	/* and we release it from the echo_skb buffer
	 * NOTE: this is one place where packet delivery will not
	 * be ordered, as we do not have any timing information
	 * when this occurred
	 */
	can_get_echo_skb(net, fifo);

	/* also mark fifo as provisioned */
	cpriv->fifos.tx_queue->pending_fifos &= ~BIT(fifo);

	spin_unlock_irqrestore(&cpriv->fifos.tx_queue->pending_lock, flags);

	/* but we need to run a bit of cleanup */
	cpriv->status.txif &= ~BIT(fifo);
	net->stats.tx_aborted_errors++;

	/* handle all the known cases accordingly - ignoring FIFO full */
	val &= CAN_FIFOSTA_TXABT |
		CAN_FIFOSTA_TXLARB |
		CAN_FIFOSTA_TXERR;
	switch (val) {
	case CAN_FIFOSTA_TXERR:
		/* this indicates a possible bus error */
		break;
	default:
		dev_warn_ratelimited(&spi->dev,
				     "Unknown TX-Fifo abort condition: %08x - stopping tx-queue\n",
				     val);
		break;
	}

	return 0;
}

int mcp25xxfd_can_int_handle_txatif(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	int i, fifo;
	int ret;

	/* if txatif is unset, then there are no
	 * can frames that have been transmitted
	 * and need to get reingested into the network stack
	 */
	if (!cpriv->status.txatif)
		return 0;
	cpriv->stats.int_txat_count++;

	/* process all the fifos with that flag set */
	for (i = 0, fifo = cpriv->fifos.tx.start;
	     i < cpriv->fifos.tx.count;
	     i++, fifo += cpriv->fifos.tx.increment) {
		if (cpriv->status.txatif & BIT(fifo)) {
			ret = mcp25xxfd_can_int_handle_txatif_fifo(spi,
								   fifo);
			if (ret)
				return ret;
		}
	}

	return 0;
}

/* submit the fifo back to the network stack */
int mcp25xxfd_can_submit_tx_frame(struct spi_device *spi, int fifo)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	struct mcp25xxfd_obj_tx *tx = (struct mcp25xxfd_obj_tx *)
		(cpriv->sram + cpriv->fifos.fifo_reg[fifo].offset);
	int dlc = (tx->flags & CAN_OBJ_FLAGS_DLC_MASK) >>
		CAN_OBJ_FLAGS_DLC_SHIFT;
	unsigned long flags;

	/* update counters */
	net->stats.tx_packets++;
	cpriv->fifos.tx.dlc_usage[dlc]++;
	priv->net->stats.tx_bytes += can_dlc2len(dlc);
	if (tx->flags & CAN_OBJ_FLAGS_FDF)
		cpriv->stats.tx_fd_count++;
	if (tx->flags & CAN_OBJ_FLAGS_BRS)
		cpriv->stats.tx_brs_count++;

	/* release the echo buffer */
	spin_lock_irqsave(&cpriv->fifos.tx_queue->pending_lock, flags);
	if (debug)
		dev_err(&spi->dev, "    get_echo_skb %i\n", fifo);
	can_get_echo_skb(priv->net, fifo);
	spin_unlock_irqrestore(&cpriv->fifos.tx_queue->pending_lock, flags);

	return 0;
}

static int mcp25xxfd_can_int_handle_tefif_fifo(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	struct mcp25xxfd_obj_tef *tef;
	u32 tef_offset = cpriv->fifos.tef.index * cpriv->fifos.tef.size;
	int fifo, ret;

	/* read the next TEF entry to get the transmit timestamp and fifo */
	tef = (struct mcp25xxfd_obj_tef *)(cpriv->sram + tef_offset);
	ret = mcp25xxfd_cmd_read_regs(spi, MCP25XXFD_SRAM_ADDR(tef_offset),
				      &tef->id, sizeof(*tef));
	if (ret)
		return ret;

	/* now we can schedule the fifo for echo submission */
	fifo = (tef->flags & CAN_OBJ_FLAGS_SEQ_MASK) >>
		CAN_OBJ_FLAGS_SEQ_SHIFT;
	mcp25xxfd_can_queue_frame(cpriv, -fifo, tef->ts);

	/* increment the tef index with wraparround */
	cpriv->fifos.tef.index++;
	if (cpriv->fifos.tef.index >= cpriv->fifos.tef.count)
		cpriv->fifos.tef.index = 0;

	/* finally just increment the TEF pointer */
	return mcp25xxfd_cmd_write_mask(spi, CAN_TEFCON,
				 CAN_TEFCON_UINC,
				 CAN_TEFCON_UINC);
}

static
int mcp25xxfd_can_int_handle_tefif_conservative(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	int ret;

	netdev_err(net,
		   "Something is wrong - we got a TEF interrupt but we were not able to detect a finished fifo\n");

	mcp25xxfd_can_tx_queue_manage(net, TX_QUEUE_STATE_STOPPED);

	/* read a single TEF - just in case */
	ret = mcp25xxfd_can_int_handle_tefif_fifo(spi);

	/* and return */
	return ret;
}

static
int mcp25xxfd_can_int_handle_tefif_oportunistic(struct spi_device *spi,
						u32 finished)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	int i, fifo, ret;

	/* now iterate those */
	for (i = 0, fifo = cpriv->fifos.tx.start;
	     i < cpriv->fifos.tx.count;
	     i++, fifo += cpriv->fifos.tx.increment) {
		if (finished & BIT(fifo)) {
			ret = mcp25xxfd_can_int_handle_tefif_fifo(spi);
			if (ret)
				return ret;
		}
	}

	return 0;
}

int mcp25xxfd_can_int_handle_tefif(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	unsigned long flags;
	u32 finished;

	if (!(cpriv->status.intf & CAN_INT_TEFIF))
		return 0;
	cpriv->stats.int_tef_count++;

	/* compute finished fifos and clear them immediately */
	spin_lock_irqsave(&cpriv->fifos.tx_queue->pending_lock, flags);
	finished = (cpriv->fifos.tx_queue->pending_fifos ^
		    cpriv->status.txreq) &
		cpriv->fifos.tx_queue->pending_fifos;

	if (debug) {
		dev_err(&spi->dev, "finished: %08x - %08x - %08x\n",
			cpriv->fifos.tx_queue->pending_fifos,
			cpriv->status.txreq,
			finished);
	}

	cpriv->fifos.tx_queue->pending_fifos &= ~finished;
	spin_unlock_irqrestore(&cpriv->fifos.tx_queue->pending_lock, flags);

	/* in case of a strange situation run in safe mode */
	if (!finished)
		return mcp25xxfd_can_int_handle_tefif_conservative(spi);

	/* otherwise run in oportunistic mode */
	return mcp25xxfd_can_int_handle_tefif_oportunistic(spi, finished);
}

static
void mcp25xxfd_can_tx_fill_fifo_common(struct net_device *net,
				       struct mcp2517fd_tx_spi_message *smsg,
				       struct mcp25xxfd_obj_tx *tx,
				       int dlc, u8 *data)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	int len = can_dlc2len(dlc);

	/* update statistics */
	cpriv->fifos.tx.dlc_usage[dlc]++;
	cpriv->fifos.fifo_info[smsg->fifo].use_count++;

	/* add fifo number as seq */
	tx->flags |= smsg->fifo << CAN_OBJ_FLAGS_SEQ_SHIFT;

	/* copy data to tx->data for future reference */
	memcpy(tx->data, data, len);

	/* transform header to controller format */
	mcp25xxfd_convert_from_cpu(&tx->id, sizeof(*tx) / sizeof(u32));

	/* copy header + data to final location - we are not aligned */
	memcpy(smsg->fill_fifo.data.header, &tx->id, sizeof(*tx) + len);

	/* convert it back to CPU format */
	mcp25xxfd_convert_to_cpu(&tx->id, sizeof(*tx) / sizeof(u32));

	/* set up size of transfer */
	smsg->fill_fifo.xfer.len = sizeof(smsg->fill_fifo.data.cmd) +
		sizeof(smsg->fill_fifo.data.header) + len;
}

static
void mcp25xxfd_can_tx_fill_fifo_fd(struct net_device *net,
				   struct canfd_frame *frame,
				   struct mcp2517fd_tx_spi_message *smsg,
				   struct mcp25xxfd_obj_tx *tx)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	int dlc = can_len2dlc(frame->len);

	/* update some statistics */
	cpriv->stats.tx_fd_count++;

	/* compute can id */
	mcp25xxfd_canid_to_mcpid(frame->can_id, &tx->id, &tx->flags);

	/* setup flags */
	tx->flags |= dlc << CAN_OBJ_FLAGS_DLC_SHIFT;
	tx->flags |= (frame->can_id & CAN_EFF_FLAG) ? CAN_OBJ_FLAGS_IDE : 0;
	tx->flags |= (frame->can_id & CAN_RTR_FLAG) ? CAN_OBJ_FLAGS_RTR : 0;
	if (frame->flags & CANFD_BRS) {
		tx->flags |= CAN_OBJ_FLAGS_BRS;
		cpriv->stats.tx_brs_count++;
	}
	tx->flags |= (frame->flags & CANFD_ESI) ? CAN_OBJ_FLAGS_ESI : 0;
	tx->flags |= CAN_OBJ_FLAGS_FDF;

	/* and do common processing */
	mcp25xxfd_can_tx_fill_fifo_common(net, smsg, tx,
					  dlc, frame->data);
}

static
void mcp25xxfd_can_tx_fill_fifo(struct net_device *net,
				struct can_frame *frame,
				struct mcp2517fd_tx_spi_message *smsg,
				struct mcp25xxfd_obj_tx *tx)
{
	/* set frame to valid dlc */
	if (frame->can_dlc > 8)
		frame->can_dlc = 8;

	/* compute can id */
	mcp25xxfd_canid_to_mcpid(frame->can_id, &tx->id, &tx->flags);

	/* setup flags */
	tx->flags |= frame->can_dlc << CAN_OBJ_FLAGS_DLC_SHIFT;
	tx->flags |= (frame->can_id & CAN_EFF_FLAG) ? CAN_OBJ_FLAGS_IDE : 0;
	tx->flags |= (frame->can_id & CAN_RTR_FLAG) ? CAN_OBJ_FLAGS_RTR : 0;

	/* and do common processing */
	mcp25xxfd_can_tx_fill_fifo_common(net, smsg, tx,
					  frame->can_dlc, frame->data);
}

/* submit the can message to the can-bus */
netdev_tx_t mcp25xxfd_can_start_xmit(struct sk_buff *skb,
				     struct net_device *net)
{
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	struct mcp25xxfd_priv *priv = cpriv->priv;
	struct spi_device *spi = priv->spi;
	struct mcp2517fd_tx_spi_message *smsg;
	struct mcp25xxfd_obj_tx *tx;
	int addr;
	unsigned long flags;
	int ret;

	/* invalid skb we can ignore */
	if (can_dropped_invalid_skb(net, skb))
		return NETDEV_TX_OK;

	/* acquire lock */
	spin_lock_irqsave(&cpriv->fifos.tx_queue->lock, flags);

	/* get the first entry */
	smsg = list_first_entry_or_null(&cpriv->fifos.tx_queue->idle,
					struct mcp2517fd_tx_spi_message,
					list);
	if (!smsg)
		goto out_busy;

	/* move from idle to queued */
	list_move_tail(&smsg->list, &cpriv->fifos.tx_queue->queued);

	/* if queue is empty then stop the network queue immediately */
	if (list_empty(&cpriv->fifos.tx_queue->idle))
		mcp25xxfd_can_tx_queue_manage_nolock(net,
						     TX_QUEUE_STATE_RUNABLE);

	/* compute the fifo in sram */
	addr = cpriv->fifos.fifo_reg[smsg->fifo].offset;
	tx = (struct mcp25xxfd_obj_tx *)(cpriv->sram + addr);

	/* fill in message from skb->data depending on can2.0 or canfd */
	if (can_is_canfd_skb(skb))
		mcp25xxfd_can_tx_fill_fifo_fd(net,
					      (struct canfd_frame *)skb->data,
					      smsg, tx);
	else
		mcp25xxfd_can_tx_fill_fifo(net,
					   (struct can_frame *)skb->data,
					   smsg, tx);

	/* submit the message asyncronously */
	ret = spi_async(spi, &smsg->msg);
	if (ret) {
		netdev_err(net, "spi_async submission of fifo %i failed - %i\n",
			   smsg->fifo, ret);
		goto out_busy;
	}

	/* keep it for reference until the message really got transmitted */
	if (debug)
		dev_err(&spi->dev, "put_echo_skb %i\n", smsg->fifo);
	can_put_echo_skb(skb, net, smsg->fifo);

	/* unlock and return */
	spin_unlock_irqrestore(&cpriv->fifos.tx_queue->lock, flags);

	return NETDEV_TX_OK;

out_busy:
	/* stop the queue */
	mcp25xxfd_can_tx_queue_manage_nolock(net, TX_QUEUE_STATE_STOPPED);

	spin_unlock_irqrestore(&cpriv->fifos.tx_queue->lock, flags);

	return NETDEV_TX_BUSY;
}
