/*
 * nonpreemp.c
 *
 * Created: 11.09.2015 18:00:29
 *  Author: HSO
 */ 

#include "nonpreemp.h"

int schedAddTask(void (*f)(void),int d, int p){
unsigned     int x;
	for(x=0;Task_List[x].func;x++);
		if(x>=TASK_SIZE){
			return -1;
		}
	Task_List[x].period=p;
	Task_List[x].delay=d;
	Task_List[x].exec=0;
	Task_List[x].func=f;
	return x;
}


/************************************************************************/
/* @shedule task                                                                     */
/************************************************************************/
void schedSchedule(void){
unsigned int x;
	for(x=0;x<TASK_SIZE;x++){
		if(!Task_List[x].func)
		continue;
		if(Task_List[x].delay){
			Task_List[x].delay--;
		}
		else{
			/*Schedule Task*/
			Task_List[x].exec++;
			Task_List[x].delay=Task_List[x].period;
		}
	}
}


/************************************************************************/
/* DISPACH                                                                     */
/************************************************************************/
void schedDispatch(void){
unsigned     int x;
	for(x=0;x<TASK_SIZE;x++){
	if(Task_List[x].exec){
		Task_List[x].exec--;
		Task_List[x].func(0);
		/*Delete task if one-shot*/
		if(!Task_List[x].period)
		Task_List[x].func=0;
		Task_List[x].exec=0;
		}
	}
}