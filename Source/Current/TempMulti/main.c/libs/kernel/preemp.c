/*
 * preemp.c
 *
 * Created: 11.09.2015 18:28:45
 *  Author: HSO
 */ 

#include "preemp.h"

void saveContext(void);
void restoreContext(void);
void enableInterrupts(void);
void disableInterrupts(void);

void schedScheduler() {
	unsigned char i;
	saveContext(); //save temporary registers, if necessary
	for (i=0; i < tasksNum; ++i) { // Heart of scheduler code
		if ( (tasks[i].elapsedTime >= tasks[i].period) // Task ready
		&& (runningTasks[currentTask] > i) // priority > current priority
		&& (!tasks[i].running)) { // Task not running (no self-preemption)
			
			disableInterrupts();  // Critical section
			tasks[i].elapsedTime = 0; // Reset time since last tick
			tasks[i].running = 1;  // Mark as running
			currentTask += 1;
			runningTasks[currentTask] = i; // Add to runningTasks
			enableInterrupts();  // End critical section
			
			tasks[i].TickFct(); // Execute tick
			
			disableInterrupts();  // Critical section
			tasks[i].running = 0;
			runningTasks[currentTask] = idleTask;  // Remove from runningTasks
			currentTask -= 1;
			enableInterrupts();  // End critical section
		}
		tasks[i].elapsedTime += tasksPeriodGCD;
	}
	restoreContext();
}

