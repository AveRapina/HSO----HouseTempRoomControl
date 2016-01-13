/*
 * nonpreemp.h
 *
 * Created: 11.09.2015 18:00:45
 *  Author: HSO
 */ 


#ifndef NONPREEMP_H_
#define NONPREEMP_H_

// MAX NUMBER OF TASK TO EXECUTE
#define TASK_SIZE 16

// SCHEDULER STRUCT
typedef struct{
	int period;
	int delay;
	void (*func)(void*);
	int exec;
}Sched_Task_t;

// ALLOCATE
Sched_Task_t Task_List[TASK_SIZE];



// Prototypes
void schedSchedule(void);
void schedDispatch(void);
int schedAddTask(void (*f)(void),int d, int p);



/* TODO ON MAIN

void task1(void){
	// work todo
}

void task2(void){
	// work todo
}

Sched_Init();
// periodic task 
FuncX_init();
Sched_AddT(&task1, 0, 4);
// one-shot task //
FuncY_init();
Sched_AddT(&task2, 50, 0);
while (1) {
	Sched_Dispatch();
};

/* ON TIMER INTERRUPT
Sched_Schedule();

*/



#endif /* NONPREEMP_H_ */