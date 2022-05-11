#include <stdio.h>
#include <stdlib.h>
#include "queue.h"

int empty(struct queue_t * q) { 
	return (q->size == 0);
}

void enqueue(struct queue_t * q, struct pcb_t * proc) {
	/* TODO: put a new process to queue [q] */	
	 if (empty(q)){
		 q->proc[0] = proc;
		 q->size += 1;
	 }
	 else if (q->size = MAX_QUEUE_SIZE){
		 // TODO: Neu het cho thi lam gi ta
	 } 
	 else{
		 q->proc[q->size] = proc;
		 q->size += 1;
	 }
}

struct pcb_t * dequeue(struct queue_t * q) {

	/* TODO: return a pcb whose prioprity is the highest
	 * in the queue [q] and remember to remove it from q
	 * */
	// return proc co prority cao nhat bang cach search / sap xep queue
	if (!empty(q)){//Neu q ko trong nghia la co proc
		uint32_t highest = q->proc[0]->priority;
		uint32_t high_pid = q->proc[0]->pid;
		size_t ind = 0;

		//Queue FIFO so when you reach the highest first time, that mean it go out first
		//Dont need to consider another highest in the queue
		for( size_t i = 1;i < q->size; i++){
			if(highest < q->proc[i]->priority){
				
				highest = q->proc[i]->priority;
				high_pid = q->proc[i]->pid;
				ind = i;
			}
		}
		return q->proc[ind];
	}
	return NULL;
}

