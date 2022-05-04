
#include <stdio.h>
#include <stdlib.h>

#include "threads/thread.h"
#include "threads/synch.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/map.h"
#include "projects/crossroads/ats.h"



/* path. A:0 B:1 C:2 D:3 */
const struct position vehicle_path[4][4][10] = {
	/* from A */ {
		/* to A */
		{{-1,-1},},
		/* to B */
		{{4,0},{4,1},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{4,0},{4,1},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{4,0},{4,1},{4,2},{4,3},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1}}
	},
	/* from B */ {
		/* to A */
		{{6,4},{5,4},{4,4},{3,4},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1}},
		/* to B */
		{{-1,-1},},
		/* to C */
		{{6,4},{5,4},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{6,4},{5,4},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
	},
	/* from C */ {
		/* to A */
		{{2,6},{2,5},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{2,6},{2,5},{2,4},{2,3},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1}},
		/* to C */
		{{-1,-1},},
		/* to D */
		{{2,6},{2,5},{2,4},{1,4},{0,4},{-1,-1},}
	},
	/* from D */ {
		/* to A */
		{{0,2},{1,2},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{0,2},{1,2},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{0,2},{1,2},{2,2},{3,2},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1}},
		/* to D */
		{{-1,-1},}
	}
};

static int is_position_outside(struct position pos)
{
	return (pos.row == -1 || pos.col == -1);
}

static int is_position_crossroad(struct position pos)
{
	return (2 <= pos.row && pos.row <= 4) && (2 <= pos.col && pos.col <= 4);
}


static int try_lock_crossroad(struct position pos_cur, int start, int dest, int step, struct vehicle_info *vi)
{
	struct position pos_next = vehicle_path[start][dest][step+1];

	if (!is_position_crossroad(pos_cur)) {
		/* next is not crossroad: we came to the point where all crossroad path locked */
		return 1;
	}

	if (lock_try_acquire(&vi->map_locks[pos_cur.row][pos_cur.col])) {
		/* got current position lock */
		if (try_lock_crossroad(pos_next, start, dest, step+1, vi)) {
			/* got next position lock */
			return 1;
		} else {
			/* failed next lock: release all */
			lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
			return 0;
		}
	} else {
		/* failed to got current lock: tell caller to release all */
		return 0;
	}
}

static void release_lock_crossroad(struct position pos_cur, int start, int dest, int step, struct vehicle_info *vi)
{
	struct position pos_prev = vehicle_path[start][dest][step-1];

	/* release cur to all prev crossroad path */
	if (is_position_crossroad(pos_cur)) {
		/* release current path */
		if (lock_held_by_current_thread(&vi->map_locks[pos_cur.row][pos_cur.col])) {
			lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
		}
		/* release prev path */
		release_lock_crossroad(pos_prev, start, dest, step-1, vi);
	}
}



/* return 0:termination, 1:success, -1:fail */
static int try_move(int start, int dest, struct vehicle_info *vi)
{
	struct position pos_cur, pos_next;
	int is_can_lock; 

	pos_next = vehicle_path[start][dest][vi->step];
	pos_cur = vi->position;

	if (vi->state == VEHICLE_STATUS_RUNNING) {
		/* check termination */
		if (is_position_outside(pos_next)) {
			/* release previous */
			lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
			/* actual move */
			vi->position.row = vi->position.col = -1;
			
			return 0;
		}
	}

	/* lock crossroad path (try to get into crossroad) */
	if (!is_position_crossroad(pos_cur) && is_position_crossroad(pos_next)) {
		/* try lock path */
		is_can_lock = try_lock_crossroad(pos_next, start, dest, vi->step, vi);
		/* check proceed or wait */
		if (is_can_lock) {
			/* release cur and proceed to crossroad */
			lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
			/* update position */
			vi->position = pos_next;
			/* check moved */
			vi->step++;
		} else {
			/* wait, do not release cur */
		}
		return 1;
	} else if (is_position_crossroad(pos_cur) && is_position_crossroad(pos_next)) { /* crossroad to crossroad (do not release lock) */
		vi->position = pos_next;
		/* check moved */
		vi->step++;
		return 1;
	}else if (is_position_crossroad(pos_cur) && !is_position_crossroad(pos_next)) { /* about to out crossroad */
		/* lock next */
		lock_acquire(&vi->map_locks[pos_next.row][pos_next.col]);
		/* release crossroad path */
		release_lock_crossroad(pos_cur, start, dest, vi->step-1, vi);
		/* proceed */
		vi->position = pos_next;
		/* check moved */
		vi->step++;
		return 1;
	}else {
		/* normal path to normal path */
		if (lock_try_acquire(&vi->map_locks[pos_next.row][pos_next.col])) {
			if (vi->state == VEHICLE_STATUS_READY) {
				/* start this vehicle */
				vi->state = VEHICLE_STATUS_RUNNING;
			} else {
				/* release current position */
				lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
			}
			/* update position */
			vi->position = pos_next;
			/* check moved */
			vi->step++;
			return 1;
		} else {
			return 2; 
		}
	}
	return 1;
}



static struct condition waiters;
static struct lock wait_lock; 
static int waitcnt; 
static int threadcnt;

void init_on_mainthread(int thread_cnt){
	/* Called once before spawning threads */
	cond_init(&waiters);
	lock_init(&wait_lock);
	waitcnt = 0;
	threadcnt =0;
	threadcnt = thread_cnt;
}

void vehicle_main_process(int start, int dest, void *_vi){
	struct vehicle_info *vi = _vi;
	int i, res;
	while (1) {
		/* vehicle main code */
		res = try_move(start, dest, vi);
		if (res == 2) {
			thread_yield();
			res = try_move(start, dest, vi);
		}

		/* wait for monitor */
		lock_acquire(&wait_lock);


		/* synchronization code using monitor */
		if (waitcnt == threadcnt-1) {
			/* proceed step */
			crossroads_step++;
			/* unitstep change! */
			unitstep_changed();
			/* wait thread list is full: wake all */
			cond_broadcast(&waiters, &wait_lock);
		} else {
			/* wait thread list is not full: I'll wait */
			waitcnt++;
			cond_wait(&waiters, &wait_lock);
			waitcnt--;
		}

		/* termination condition. */ 
		if (res == 0) {
			/* decrease thread count */
			threadcnt--;
			lock_release(&wait_lock);
			break;
		}
		lock_release(&wait_lock);
		thread_yield(); 
	}	
}

void vehicle_loop(void *_vi)
{
	
	int start, dest;
	struct vehicle_info *vi = _vi;
	

	start = vi->start - 'A';
	dest = vi->dest - 'A';

	vi->position.row = vi->position.col = -1;
	vi->state = VEHICLE_STATUS_READY;
	vi->step = 0;

	//vehicle_main_process(start, dest, &_vi);	
		int i, res;
	while (1) {
		/* vehicle main code */
		res = try_move(start, dest, vi);
		if (vi->state == VEHICLE_STATUS_READY) {
			for (i=0; i<100; i++) {
				res = try_move(start, dest, vi);
				thread_yield();
			}
		} 
		/* wait for monitor */
		lock_acquire(&wait_lock);


		/* synchronization code using monitor */
		if (waitcnt == threadcnt-1) {
			/* proceed step */
			crossroads_step++;
			/* unitstep change! */
			unitstep_changed();
			/* wait thread list is full: wake all */
			cond_broadcast(&waiters, &wait_lock);
		} else {
			/* wait thread list is not full: I'll wait */
			waitcnt++;
			cond_wait(&waiters, &wait_lock);
			waitcnt--;
		}

		/* termination condition. */ 
		if (res == 0) {
			/* decrease thread count */
			threadcnt--;
			lock_release(&wait_lock);
			break;
		}
		lock_release(&wait_lock);
		thread_yield(); 
	}	

	/* status transition must happen before sema_up */
	vi->state = VEHICLE_STATUS_FINISHED;
}
