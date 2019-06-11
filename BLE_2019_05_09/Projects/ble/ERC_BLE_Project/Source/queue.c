#include "queue.h"

void initQueue(queueType *q)
{
  q->front = 0;
  q->rear  = 0;
}

int8 isEmpty(queueType *q)
{
  return (q->front == q->rear);
}

int8 isFull(queueType *q)
{
  return ((q->rear+1) % MAX_QUEUE_SIZE == q->front);
}

void enQueue(queueType *q, int8 data)
{
  if(isFull(q)){
    int dummy;
    dummy = deQueue(q);
    q->rear = (q->rear+1) % MAX_QUEUE_SIZE;
    q->queueBuff[q->rear] = data;
    
    return ;
  }
  q->rear = (q->rear+1) % MAX_QUEUE_SIZE;
  q->queueBuff[q->rear] = data;
}

int8 deQueue(queueType *q)
{
  if(isEmpty(q))
    return -1;
  
  q->front = (q->front+1) % MAX_QUEUE_SIZE;
  return q->queueBuff[q->front];
}

int8 peekQueue(queueType *q)
{
  if(isEmpty(q))
    return -1;
  
  return q->queueBuff[q->rear];
}
