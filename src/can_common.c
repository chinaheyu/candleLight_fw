/*

The MIT License (MIT)

Copyright (c) 2016 Hubert Denkmair

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#include "can_common.h"
#include "led.h"
#include "timer.h"
#include "usbd_gs_can.h"

#ifndef CONFIG_CANFD
const struct gs_device_bt_const_extended CAN_btconst_ext;
#endif

int can_check_bittiming(const struct can_bittiming_const *btc,
						const struct gs_device_bittiming *timing)
{
	const uint32_t tseg1 = timing->prop_seg + timing->phase_seg1;

	if (tseg1 < btc->tseg1_min ||
		tseg1 > btc->tseg1_max ||
		timing->phase_seg2 < btc->tseg2_min ||
		timing->phase_seg2 > btc->tseg2_max ||
		timing->sjw > btc->sjw_max ||
		timing->brp < btc->brp_min ||
		timing->brp > btc->brp_max)
		return -1;

	return 0;
}

#ifdef BOARD_SCUT_candleLightFD
void CAN_EchoFrame(USBD_GS_CAN_HandleTypeDef *hcan, can_data_t *channel)
{
	bool was_irq_enabled;
	struct gs_host_frame_object *frame_object;
	struct gs_host_frame *frame = NULL;
	struct list_head *p, *n;
	FDCAN_TxEventFifoTypeDef tx_event;

	if (HAL_FDCAN_GetTxEvent(&channel->channel, &tx_event) == HAL_OK) {
		was_irq_enabled = disable_irq();
		list_for_each_safe(p, n, &channel->list_echo) {
			frame_object = list_entry(p, struct gs_host_frame_object, list);
			list_del(p);
			if (frame_object->frame.reserved == tx_event.MessageMarker) {
				frame = &frame_object->frame;
				break;
			} else {
				list_add_tail(&frame_object->list, &hcan->list_frame_pool);
			}
		}
		restore_irq(was_irq_enabled);

		if (!frame)
			return;

		frame->reserved = 0;

		if (IS_ENABLED(CONFIG_CANFD) && frame->flags & GS_CAN_FLAG_FD)
			frame->canfd_ts->timestamp_us = timer_get();
		else
			frame->classic_can_ts->timestamp_us = timer_get();

		list_add_tail_locked(&frame_object->list, &hcan->list_to_host);
		led_indicate_trx(&channel->leds, LED_TX);
	}
}
#endif

void CAN_SendFrame(USBD_GS_CAN_HandleTypeDef *hcan, can_data_t *channel)
{
#ifdef BOARD_SCUT_candleLightFD
	UNUSED(hcan);
#endif

	struct gs_host_frame_object *frame_object;

	bool was_irq_enabled = disable_irq();
	frame_object = list_first_entry_or_null(&channel->list_from_host,
											struct gs_host_frame_object,
											list);
	if (!frame_object) {
		restore_irq(was_irq_enabled);
		return;
	}

	list_del(&frame_object->list);
	restore_irq(was_irq_enabled);

	struct gs_host_frame *frame = &frame_object->frame;

#ifdef BOARD_SCUT_candleLightFD
	frame->reserved = channel->echo_marker;
#endif

	if (!can_send(channel, frame)) {
		list_add_locked(&frame_object->list, &channel->list_from_host);
		return;
	}

#ifdef BOARD_SCUT_candleLightFD
	channel->echo_marker++;
#endif

	// Echo sent frame back to host
#ifdef BOARD_SCUT_candleLightFD
	if (channel->channel.Init.Mode != FDCAN_MODE_BUS_MONITORING) {
		list_add_tail_locked(&frame_object->list, &channel->list_echo);
	}
#else
	frame->reserved = 0x0;
	if (IS_ENABLED(CONFIG_CANFD) && frame->flags & GS_CAN_FLAG_FD)
		frame->canfd_ts->timestamp_us = timer_get();
	else
		frame->classic_can_ts->timestamp_us = timer_get();

	list_add_tail_locked(&frame_object->list, &hcan->list_to_host);

	led_indicate_trx(&channel->leds, LED_TX);
#endif
}

void CAN_ReceiveFrame(USBD_GS_CAN_HandleTypeDef *hcan, can_data_t *channel)
{
	struct gs_host_frame_object *frame_object;

	if (!can_is_rx_pending(channel)) {
		return;
	}

	bool was_irq_enabled = disable_irq();
	frame_object = list_first_entry_or_null(&hcan->list_frame_pool,
											struct gs_host_frame_object,
											list);
	if (!frame_object) {
		restore_irq(was_irq_enabled);
		return;
	}

	list_del(&frame_object->list);
	restore_irq(was_irq_enabled);

	struct gs_host_frame *frame = &frame_object->frame;

	if (!can_receive(channel, frame)) {
		list_add_tail_locked(&frame_object->list, &hcan->list_frame_pool);
		return;
	}

	frame->echo_id = 0xFFFFFFFF; // not an echo frame
	frame->reserved = 0;

	list_add_tail_locked(&frame_object->list, &hcan->list_to_host);

	led_indicate_trx(&channel->leds, LED_RX);
}

void CAN_HandleError(USBD_GS_CAN_HandleTypeDef *hcan, can_data_t *channel)
{
	struct gs_host_frame_object *frame_object;

	uint32_t curr_err = can_get_error_status(channel);

	bool was_irq_enabled = disable_irq();
	can_manage_bus_off_recovery(channel, curr_err);
	restore_irq(was_irq_enabled);

	// If there are frames to receive, don't report any error frames. The
	// best we can localize the errors to is "after the last successfully
	// received frame", so wait until we get there. LEC will hold some error
	// to report even if multiple pass by.
	if (can_is_rx_pending(channel)) {
		return;
	}

	if (!can_has_error_status_changed(channel->last_err, curr_err)) {
		// No change in error status, nothing to report
		goto no_change;
	}

	was_irq_enabled = disable_irq();
	frame_object = list_first_entry_or_null(&hcan->list_frame_pool,
											struct gs_host_frame_object,
											list);
	if (!frame_object) {
		restore_irq(was_irq_enabled);
		return;
	}

	list_del(&frame_object->list);
	restore_irq(was_irq_enabled);

	struct gs_host_frame *frame = &frame_object->frame;
	frame->classic_can_ts->timestamp_us = timer_get();
	frame->channel = channel->nr;

	if (can_parse_error_status(channel, frame, channel->last_err, curr_err)) {
		list_add_tail_locked(&frame_object->list, &hcan->list_to_host);
	} else {
		list_add_tail_locked(&frame_object->list, &hcan->list_frame_pool);
	}

no_change:
	channel->last_err = curr_err;
}
