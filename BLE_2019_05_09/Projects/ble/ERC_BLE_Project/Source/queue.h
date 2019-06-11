#include "hal_types.h"

#define MAX_QUEUE_SIZE                                                          101

typedef struct queueS{
  int8 queueBuff[MAX_QUEUE_SIZE];
  int8 rear;
  int8 front;
}queueType;

void initQueue(queueType *q);
int8 isEmpty(queueType *q);
int8 isFull(queueType *q);
void enQueue(queueType *q, int8 data);
int8 deQueue(queueType *q);
int8 peekQueue(queueType *q);