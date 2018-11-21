// SPDX-License-Identifier: GPL-2.0

/* CAN bus driver for Microchip 25XXFD CAN Controller with SPI Interface
 * implementation of can interrupt handling
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
#include <linux/netdevice.h>
#include <linux/slab.h>
#include <linux/sort.h>

static void mcp25xxfd_can_error_skb(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	struct sk_buff *skb;
	struct can_frame *frame;

	/* allocate error frame */
	skb = alloc_can_err_skb(net, &frame);
	if (!skb) {
		netdev_err(net, "cannot allocate error skb\n");
		return;
	}

	/* setup can error frame data */
	frame->can_id |= cpriv->error_frame.id;
	memcpy(frame->data, cpriv->error_frame.data, sizeof(*frame->data));

	/* and submit it */
	netif_receive_skb(skb);
}

static int mcp25xxfd_can_int_clear_int_flags(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	u32 clearable_irq_active = cpriv->status.intf & CAN_INT_IF_CLEAR_MASK;
	u32 clear_irq = cpriv->status.intf & (~CAN_INT_IF_CLEAR_MASK);

	/* if no clearable flags are set then skip the whole transfer */
	if (!clearable_irq_active)
		return 0;

	return mcp25xxfd_cmd_write_mask(spi, CAN_INT,
					clear_irq, clearable_irq_active);
}

static int mcp25xxfd_can_ist_handle_serrif_txmab(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);

	net->stats.tx_fifo_errors++;
	net->stats.tx_errors++;
	cpriv->stats.int_serr_tx_count++;

	return 0;
}

static int mcp25xxfd_can_ist_handle_serrif_rxmab(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);

	net->stats.rx_dropped++;
	net->stats.rx_errors++;
	cpriv->stats.int_serr_rx_count++;

	return 0;
}

static int mcp25xxfd_can_int_handle_serrif(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	u32 clear;
	int ret;

	if (!(cpriv->status.intf & CAN_INT_SERRIF))
		return 0;

	/* increment statistics counter now */
	cpriv->stats.int_serr_count++;

	/* clear some interrupts immediately,
	 * so that we get notified if they happen again
	 */
	clear = CAN_INT_SERRIF | CAN_INT_MODIF | CAN_INT_IVMIF;
	ret = mcp25xxfd_cmd_write_mask(spi, CAN_INT,
				       cpriv->status.intf & (~clear),
				       clear);
	if (ret)
		return ret;

	/* Errors here are:
	 * * Bus Bandwidth Error: when a RX Message Assembly Buffer
	 *   is still full when the next message has already arrived
	 *   the recived message shall be ignored
	 * * TX MAB Underflow: when a TX Message is invalid
	 *   due to ECC errors or TXMAB underflow
	 *   in this situatioon the system will transition to
	 *   Restricted or Listen Only mode
	 */

	cpriv->error_frame.id |= CAN_ERR_CRTL;
	cpriv->error_frame.data[1] |= CAN_ERR_CRTL_UNSPEC;

	/* a mode change + invalid message would indicate
	 * TX MAB Underflow
	 */
	if ((cpriv->status.intf & CAN_INT_MODIF) &&
	    (cpriv->status.intf & CAN_INT_IVMIF)) {
		return mcp25xxfd_can_ist_handle_serrif_txmab(spi);
	}

	/* for RX there is only the RXIF an indicator
	 * - surprizingly RX-MAB does not change mode or anything
	 */
	if (cpriv->status.intf & CAN_INT_RXIF)
		return mcp25xxfd_can_ist_handle_serrif_rxmab(spi);

	/* the final case */
	dev_warn_ratelimited(&spi->dev,
			     "unidentified system error - intf =  %08x\n",
			     cpriv->status.intf);

	return 0;
}

static int mcp25xxfd_can_int_handle_modif(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	int mode;
	int ret;

	/* Note that this irq does not get triggered in all situations
	 * for example SERRIF will move to RESTICTED or LISTENONLY
	 * but MODIF will not be raised!
	 */

	if (!(cpriv->status.intf & CAN_INT_MODIF))
		return 0;
	cpriv->stats.int_mod_count++;

	/* get the current mode */
	ret = mcp25xxfd_can_get_mode(spi, &mode);
	if (ret)
		return ret;
	mode = ret;

	/* switches to the same mode as before are ignored
	 * - this typically happens if the driver is shortly
	 *   switching to a different mode and then returning to the
	 *   original mode
	 */
	if (mode == cpriv->mode)
		return 0;

	/* if we are restricted, then return to "normal" mode */
	if (mode == CAN_CON_MODE_RESTRICTED) {
		cpriv->mode = mode;
		return mcp25xxfd_can_switch_mode(spi, &cpriv->regs.con,
						 (net->mtu == CAN_MTU) ?
						 CAN_CON_MODE_CAN2_0 :
						 CAN_CON_MODE_MIXED);
	}

	/* the controller itself will transition to sleep, so we ignore it */
	if (mode == CAN_CON_MODE_SLEEP) {
		cpriv->mode = mode;
		return 0;
	}

	/* these we need to handle correctly, so warn and give context */
	dev_warn(&spi->dev,
		 "Controller unexpectedly switched from mode %u to %u\n",
		 cpriv->mode, mode);

	/* assign the mode as current */
	cpriv->mode = mode;

	return 0;
}

static int mcp25xxfd_compare_obj_ts(const void *a, const void *b)
{
	s32 ats = ((struct mcp25xxfd_obj_ts *)a)->ts;
	s32 bts = ((struct mcp25xxfd_obj_ts *)b)->ts;

	if (ats < bts)
		return -1;
	if (ats > bts)
		return 1;
	return 0;
}

static int mcp25xxfd_can_submit_frames(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	struct mcp25xxfd_obj_ts *queue = cpriv->fifos.submit_queue;
	int count = cpriv->fifos.submit_queue_count;
	int i, fifo;
	int ret;

	/* skip processing if the queue count is 0 */
	if (count == 0)
		goto out;

	/* sort the fifos (rx and TEF) by receive timestamp */
	sort(queue, count, sizeof(*queue), mcp25xxfd_compare_obj_ts, NULL);

	/* now submit the fifos  */
	for (i = 0; i < count; i++) {
		fifo = queue[i].fifo;
		if (fifo > 0)
			ret = mcp25xxfd_can_submit_rx_frame(spi, fifo);
		else
			ret = mcp25xxfd_can_submit_tx_frame(spi, -fifo);
		if (ret)
			return ret;
	}

out:
	/* enable tx_queue if necessary */
	mcp25xxfd_can_tx_queue_restart(net);

	return 0;
}

static int mcp25xxfd_can_int_handle_rxif(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);

	if (!cpriv->status.rxif)
		return 0;

	cpriv->stats.int_rx_count++;

	/* read all the fifos */
	return mcp25xxfd_can_read_rx_frames(spi);
}

static int mcp25xxfd_can_int_handle_rxovif(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	int ret, i;

	if (!cpriv->status.rxovif)
		return 0;
	cpriv->stats.int_rxov_count++;

	/* clear all fifos that have an overflow bit set */
	for (i = 0; i < 32; i++) {
		if (cpriv->status.rxovif & BIT(i)) {
			/* clear fifo status */
			ret = mcp25xxfd_cmd_write_mask(spi,
						       CAN_FIFOSTA(i),
						       0,
						       CAN_FIFOSTA_RXOVIF);
			if (ret)
				return ret;

			/* update statistics */
			net->stats.rx_over_errors++;
			net->stats.rx_errors++;

			/* and prepare ERROR FRAME */
			cpriv->error_frame.id |= CAN_ERR_CRTL;
			cpriv->error_frame.data[1] |= CAN_ERR_CRTL_RX_OVERFLOW;
		}
	}

	return 0;
}

static int mcp25xxfd_can_int_handle_eccif(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);

	if (!(cpriv->status.intf & CAN_INT_ECCIF))
		return 0;

	cpriv->stats.int_ecc_count++;

	/* and prepare ERROR FRAME */
	cpriv->error_frame.id |= CAN_ERR_CRTL;
	cpriv->error_frame.data[1] |= CAN_ERR_CRTL_UNSPEC;

	/* delegate to interrupt cleaning */
	return mcp25xxfd_clear_ecc_interrupts(spi);
}

static int mcp25xxfd_can_int_handle_ivmif(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);

	if (!(cpriv->status.intf & CAN_INT_IVMIF))
		return 0;

	cpriv->stats.int_ivm_count++;

	/* if we have a systemerror as well,
	 * then ignore it as they correlate
	 */
	if (cpriv->status.intf & CAN_INT_SERRIF)
		return 0;

	/* otherwise it is an RX issue, so account for it here */
	cpriv->error_frame.id |= CAN_ERR_PROT;
	cpriv->error_frame.data[2] |= CAN_ERR_PROT_FORM;

	/* and update statistics */
	net->stats.rx_frame_errors++;
	net->stats.rx_errors++;

	return 0;
}

static int mcp25xxfd_can_int_handle_cerrif(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);

	if (!(cpriv->status.intf & CAN_INT_CERRIF))
		return 0;
	cpriv->stats.int_cerr_count++;

	dev_err(&spi->dev, "in %s\n", __func__);
	return 0;
}

static int mcp25xxfd_can_int_error_counters(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);

	if (cpriv->status.trec & CAN_TREC_TXWARN) {
		cpriv->bus.new_state = CAN_STATE_ERROR_WARNING;
		cpriv->error_frame.id |= CAN_ERR_CRTL;
		cpriv->error_frame.data[1] |= CAN_ERR_CRTL_TX_WARNING;
	}
	if (cpriv->status.trec & CAN_TREC_RXWARN) {
		cpriv->bus.new_state = CAN_STATE_ERROR_WARNING;
		cpriv->error_frame.id |= CAN_ERR_CRTL;
		cpriv->error_frame.data[1] |= CAN_ERR_CRTL_RX_WARNING;
	}
	if (cpriv->status.trec & CAN_TREC_TXBP) {
		cpriv->bus.new_state = CAN_STATE_ERROR_PASSIVE;
		cpriv->error_frame.id |= CAN_ERR_CRTL;
		cpriv->error_frame.data[1] |= CAN_ERR_CRTL_TX_PASSIVE;
	}
	if (cpriv->status.trec & CAN_TREC_RXBP) {
		cpriv->bus.new_state = CAN_STATE_ERROR_PASSIVE;
		cpriv->error_frame.id |= CAN_ERR_CRTL;
		cpriv->error_frame.data[1] |= CAN_ERR_CRTL_RX_PASSIVE;
	}
	if (cpriv->status.trec & CAN_TREC_TXBO) {
		cpriv->bus.new_state = CAN_STATE_BUS_OFF;
		cpriv->error_frame.id |= CAN_ERR_BUSOFF;
	}

	return 0;
}

static int mcp25xxfd_can_int_error_handling(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);

	/* based on the last state state check the new state */
	switch (cpriv->can.state) {
	case CAN_STATE_ERROR_ACTIVE:
		if (cpriv->bus.new_state >= CAN_STATE_ERROR_WARNING &&
		    cpriv->bus.new_state <= CAN_STATE_BUS_OFF)
			cpriv->can.can_stats.error_warning++;
		/* fallthrough */
	case CAN_STATE_ERROR_WARNING:
		if (cpriv->bus.new_state >= CAN_STATE_ERROR_PASSIVE &&
		    cpriv->bus.new_state <= CAN_STATE_BUS_OFF)
			cpriv->can.can_stats.error_passive++;
		break;
	default:
		break;
	}
	cpriv->can.state = cpriv->bus.new_state;

	/* and send error packet */
	if (cpriv->error_frame.id)
		mcp25xxfd_can_error_skb(spi);

	/* handle BUS OFF */
	if (cpriv->can.state == CAN_STATE_BUS_OFF) {
		if (cpriv->can.restart_ms == 0) {
			cpriv->can.can_stats.bus_off++;
			/* force exit of interrupt handler */
//			cpriv->irq.running = false;
			/* bring the state of the bus down */
			can_bus_off(net);
//			mcp25xxfd_stop_queue(spi);
//			mcp25xxfd_hw_sleep(spi);
		}
	} else {
		/* restart the tx queue if needed */
//		if (priv->fifos.tx_processed_mask == priv->fifos.tx_fifo_mask)
//			mcp25xxfd_wake_queue(spi);
	}

	return 0;
}

static int mcp25xxfd_can_int_bdiag_handling(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);

	if (!cpriv->bus.diag_clear_mask)
		return 0;

	dev_err(&spi->dev, "in %s\n", __func__);

	return mcp25xxfd_cmd_write_mask(spi,
					CAN_BDIAG1,
					cpriv->bus.diag_clear_value,
					cpriv->bus.diag_clear_mask);
}

static int mcp25xxfd_can_int_handle_status(struct spi_device *spi)
{
	struct mcp25xxfd_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	int ret;

	/* clear all the interrupts asap - we have them on file allready */
	ret = mcp25xxfd_can_int_clear_int_flags(spi);
	if (ret)
		return ret;

	/* set up new state and error frame for this loop */
	cpriv->bus.new_state = cpriv->bus.state;
	cpriv->bus.diag_clear_value = 0;
	cpriv->bus.diag_clear_mask = 0;
	memset(&cpriv->error_frame, 0, sizeof(cpriv->error_frame));

	/* setup the process queue by clearing the counter */
	cpriv->fifos.submit_queue_count = 0;

	/* handle interrupts */

	/* system error interrupt needs to get handled first
	 * to get us out of restricted mode
	 */
	ret = mcp25xxfd_can_int_handle_serrif(spi);
	if (ret)
		return ret;

	/* mode change interrupt */
	ret = mcp25xxfd_can_int_handle_modif(spi);
	if (ret)
		return ret;

	/* handle the rx */
	ret = mcp25xxfd_can_int_handle_rxif(spi);
	if (ret)
		return ret;
	/* handle aborted TX FIFOs */
	ret = mcp25xxfd_can_int_handle_txatif(spi);
	if (ret)
		return ret;

	/* handle the TEF */
	ret = mcp25xxfd_can_int_handle_tefif(spi);
	if (ret)
		return ret;

	/* handle error interrupt flags */
	ret = mcp25xxfd_can_int_handle_rxovif(spi);
	if (ret)
		return ret;

	/* sram ECC error interrupt */
	ret = mcp25xxfd_can_int_handle_eccif(spi);
	if (ret)
		return ret;

	/* message format interrupt */
	ret = mcp25xxfd_can_int_handle_ivmif(spi);
	if (ret)
		return ret;

	/* handle bus errors in more detail */
	ret = mcp25xxfd_can_int_handle_cerrif(spi);
	if (ret)
		return ret;

	/* error counter handling */
	ret = mcp25xxfd_can_int_error_counters(spi);
	if (ret)
		return ret;

	/* error counter handling */
	ret = mcp25xxfd_can_int_error_handling(spi);
	if (ret)
		return ret;

	/* clear bdiag flags */
	ret = mcp25xxfd_can_int_bdiag_handling(spi);
	if (ret)
		return ret;

	/* and submit can frames to network stack */
	ret = mcp25xxfd_can_submit_frames(spi);

	return ret;
}

irqreturn_t mcp25xxfd_can_int(int irq, void *dev_id)
{
	struct mcp25xxfd_priv *priv = dev_id;
	struct spi_device *spi = priv->spi;
	struct net_device *net = priv->net;
	struct mcp25xxfd_can_priv *cpriv = netdev_priv(net);
	int ret;

	/* count interrupt calls */
	cpriv->stats.irq_calls++;

	/* as long as we should be running */
	while (1) {
		/* count irq loops */
		cpriv->stats.irq_loops++;

		/* read interrupt status flags in bulk */
		ret = mcp25xxfd_cmd_read_regs(spi, CAN_INT,
					      &cpriv->status.intf,
					      sizeof(cpriv->status));
		if (ret)
			return ret;

		/* only act if the IE mask configured has active IF bits
		 * otherwise the Interrupt line should be deasserted already
		 * so we can exit the loop
		 */
		if (((cpriv->status.intf >> CAN_INT_IE_SHIFT) &
		       cpriv->status.intf) == 0)
			break;

		/* handle the status */
		ret = mcp25xxfd_can_int_handle_status(spi);
		if (ret)
			return ret;
	}

	return IRQ_HANDLED;
}
