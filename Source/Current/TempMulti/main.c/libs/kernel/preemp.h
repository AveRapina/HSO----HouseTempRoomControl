/*
 * preemp.h
 *
 * Created: 11.09.2015 18:28:33
 *  Author: HSO
 */ 


#ifndef PREEMP_H_
#define PREEMP_H_

unsigned char runningTasks[3] = {255};//Track running tasks-[0] always idleTask
const unsigned long idleTask = 255; // 0 highest priority, 255 lowest
unsigned char currentTask = 0; //Index of highest priority task in runningTasks

#define NUMBER_RUNNING_TASKS 3

typedef struct task {
	unsigned long period; // Rate at which the task should tick
	unsigned long elapsedTime; // Time since task's last tick
	unsigned int running;
	void (*TickFct)(void); // Function to call for task's tick
} task;

int tasksNum = 0;
task tasks[0];
unsigned char tasksPeriodGCD = 0;
unsigned char processingRdyTasks = 0;





#endif /* PREEMP_H_ */