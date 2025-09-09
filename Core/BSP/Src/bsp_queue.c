/*
 * bsp_queue.c
 *
 *  Created on: 3 Mar 2022
 *      Author: wx
 */
#include "stm32f4xx.h"
#include "bsp_queue.h"
#include <string.h>



QueueOpStat_t queue_init(queue_t* queue){
	QueueOpStat_t op_stat;
	op_stat.op_status = Q_OK;
	if (queue == NULL){
		op_stat.op_status = Q_INVALID;
		return op_stat;
	}
	queue->curr_byte_pos = 0;
	queue->last_byte_pos=0;
	queue->last_time=0;
	return op_stat;
}

uint16_t queue_get_size(queue_t* queue){
	uint16_t size = queue->last_byte_pos - queue->curr_byte_pos;
	size = (size > TQUEUE_SIZE) ? size + TQUEUE_SIZE : size;
	return size;
}

/*
 * Adds a byte to the end of the byte_queue
 * Add one byte at a time!
 */
QueueOpStat_t queue_append_byte(queue_t* queue, uint8_t data){
	QueueOpStat_t op_stat;
	op_stat.op_status = Q_OK;
	queue->last_time = HAL_GetTick();
	queue->queue[queue->last_byte_pos] = data;
	op_stat.bytes_appended = 1;
	queue->last_byte_pos = (queue->last_byte_pos >= TQUEUE_SIZE-1) ? 0 : queue->last_byte_pos + 1;
	if (queue_get_size(queue) > TQUEUE_SIZE) {
		op_stat.op_status = Q_FULL;
		queue->curr_byte_pos = (queue->curr_byte_pos == TQUEUE_SIZE-1) ? 0 : queue->curr_byte_pos+1;
		return op_stat;
	}
	return op_stat;
}

QueueOpStat_t queue_append_bytes(queue_t* queue, uint8_t *data, uint16_t len){
	uint16_t pos = 0;
	QueueOpStat_t op_stat;
	while (len > 0){
		op_stat = queue_append_byte(queue,data[pos++]);
		len--;
	}
	return op_stat;
}

uint8_t queue_pop_element(queue_t* queue){
	if (queue_get_size(queue) == 0){
		return 0;
	} else {
		queue->curr_byte_pos +=1;
		queue->curr_byte_pos = (queue->curr_byte_pos >= TQUEUE_SIZE) ?
				0:queue->curr_byte_pos;
		uint8_t temp =queue->queue[queue->curr_byte_pos];
		return temp;
	}
}


QueueOpStat_t queue_pop_elements(queue_t* queue, uint8_t* data, uint16_t len){
		QueueOpStat_t op_stat;
		uint16_t q_size = queue_get_size(queue);
		if (q_size < len){
			op_stat.op_status = Q_NOT_ENOUGH_BYTES;
			len = q_size;
		}

		uint16_t bytes_to_end = TQUEUE_SIZE-queue->curr_byte_pos;
		if (bytes_to_end >= len){
			memcpy(data, &queue->queue[queue->curr_byte_pos], len);
		} else {
			memcpy(data, &queue->queue[queue->curr_byte_pos], bytes_to_end);
			memcpy((data+bytes_to_end), &queue->queue[0],(len-bytes_to_end));
		}

		queue->curr_byte_pos += len;
		queue->curr_byte_pos = (queue->curr_byte_pos >= TQUEUE_SIZE) ?
				queue->curr_byte_pos-TQUEUE_SIZE : queue->curr_byte_pos;

	return op_stat;
}

uint8_t queue_peek(queue_t* queue){
	if (queue_get_size(queue)>0){
		return queue->queue[queue->curr_byte_pos];
	}
	else{
		return 0;
	}
}

QueueOpStat_t queue_peek_number(queue_t* queue, uint8_t* buffer,uint16_t size){
	QueueOpStat_t op_stat;
	queue->last_time = HAL_GetTick();
	op_stat.op_status = Q_OK;
	uint16_t q_size = queue_get_size(queue);
	uint16_t bytes_to_end = TQUEUE_SIZE-queue->curr_byte_pos;
	if (q_size < size){
		op_stat.op_status = Q_NOT_ENOUGH_BYTES;
		size = q_size;
	}

	if (bytes_to_end >= size){
		memcpy(buffer, &queue->queue[queue->curr_byte_pos], size);
	} else {
		memcpy(buffer, &queue->queue[queue->curr_byte_pos], bytes_to_end);
		memcpy((buffer+bytes_to_end), &queue->queue[0],(size-bytes_to_end));
	}
	op_stat.bytes_appended = size;
	return op_stat;
}


QueueOpStat_t queue_remove_number(queue_t* queue, uint16_t size){
//	queue_sanity_check(queue);
	QueueOpStat_t op_stat;
	op_stat.op_status = Q_OK;
	uint16_t q_size = queue_get_size(queue);
	if (q_size < size){
		op_stat.op_status = Q_NOT_ENOUGH_BYTES;
		size = q_size;
	}
	queue->curr_byte_pos += size;
	queue->curr_byte_pos = (queue->curr_byte_pos >= TQUEUE_SIZE) ?
			queue->curr_byte_pos-TQUEUE_SIZE : queue->curr_byte_pos;

	op_stat.bytes_appended = size;
	return op_stat;
}

uint8_t queue_sanity_check(queue_t* queue){
	//check if the queue values are still ok
	uint16_t q_size = queue_get_size(queue);
	if (q_size > TQUEUE_SIZE){
		return 1;
	}
	if (queue->last_byte_pos >= TQUEUE_SIZE || queue->last_byte_pos < 0){
		return 2;
	}
	if (queue->curr_byte_pos >= TQUEUE_SIZE || queue->curr_byte_pos < 0){
		return 3;
	}

	int16_t queue_real_size = queue->last_byte_pos - queue->curr_byte_pos;
	queue_real_size = (queue_real_size < 0) ? TQUEUE_SIZE + queue_real_size : queue_real_size;
	if (q_size== TQUEUE_SIZE){
				return 0;
	}
	if (queue_real_size != q_size){
		return 4;
	}
	return 0;
}



