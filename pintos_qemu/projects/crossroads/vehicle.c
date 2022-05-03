
#include <stdio.h>

#include "threads/thread.h"
#include "threads/synch.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/map.h"
#include "projects/crossroads/ats.h"

/*
../../utils/pintos crossroads aAC:bAC:cAC:dBD:eBD:fBD:gCA:hCA:iCA:jDB:kDB:lDB:mAB:nBC:oCD:pDA:qAD:rDC:sCB:tBA:uBA:vAD:wDC:xCB:yBC:zDA
*/

//for unit step
struct semaphore *sema;
struct condition con; 
struct lock vehicle_condition_lock; 
int movable_vehicle_count;
int remain_vehicle_count; //지금 움직이고 있는 스레드 수
int whole_vehicle_count; //전체 스레드 수
int wait_vehivle_count =0 ; // ready 상태 애들, finish 상태 애들
int lock_vehicle_count = 0;
//돌아가는 애 += lock 걸렸던 애들
//remaincount = 돌아가는애 
//기다리는애 0개로
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

/*교차로에 있는 길 풀어달라고 요청*/
static void release_lock_crossroad(struct position pos_cur, int start, int dest, int step, struct vehicle_info *vi) 
{
	//이전 좌표 받아오기
	struct position pos_prev = vehicle_path[start][dest][step-1];

	/* release cur to all prev crossroad path in crossroad*/
	if (is_position_crossroad(pos_cur)) {
		/* release current path if lock is in current thread*/
		if (lock_held_by_current_thread(&vi->map_locks[pos_cur.row][pos_cur.col])) {
			lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
		}
		/* release prev path */
		release_lock_crossroad(pos_prev, start, dest, step-1, vi);
	}
}

//교차로에 있는 길 막아버리기
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

/* return 0:termination, 1:success, -1:fail */
static int try_move(int start, int dest, struct vehicle_info *vi)
{
	struct position pos_cur, pos_next;
	int got_lock; 

	pos_next = vehicle_path[start][dest][vi->step];
	pos_cur = vi->position;

	if(start == dest){ 
		movable_vehicle_count--;
		return 0; 
	}

	if (vi->state == VEHICLE_STATUS_RUNNING) {
		/* check termination */
		if (is_position_outside(pos_next)) {
			/* actual move */
			vi->position.row = vi->position.col = -1;
			/* release previous */
			lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
			movable_vehicle_count--;
			return 0;
		}
	}

	/* lock crossroad path (try to get into crossroad) */
	if (!is_position_crossroad(pos_cur) && is_position_crossroad(pos_next)) {
		/* try lock path */
		got_lock = try_lock_crossroad(pos_next, start, dest, vi->step, vi);
		/* check proceed or wait */
		if (got_lock) {
			/* release cur and proceed to crossroad */
			lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
			/* update position */
			vi->position = pos_next;
			/* check moved */
			vi->step++;
		} else {
			/* wait, do not release cur */
		}
		goto end_turn; 
	}

	/* crossroad to crossroad (do not release lock) */
	if (is_position_crossroad(pos_cur) && is_position_crossroad(pos_next)) {
		vi->position = pos_next;
		/* check moved */
		vi->step++;

		goto end_turn; 
	}

	/* about to out crossroad */
	if (is_position_crossroad(pos_cur) && !is_position_crossroad(pos_next)) {
		/* lock next */
		lock_acquire(&vi->map_locks[pos_next.row][pos_next.col]);
		/* release crossroad path */
		release_lock_crossroad(pos_cur, start, dest, vi->step-1, vi);
		/* proceed */
		vi->position = pos_next;
		/* check moved */
		vi->step++;

		goto end_turn;
	}

	/* normal path to normal path */
	lock_acquire(&vi->map_locks[pos_next.row][pos_next.col]);
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
	
end_turn: 
	return 1;
}

void init_on_mainthread(int thread_cnt){
	/* Called once before spawning threads */
	lock_init(&vehicle_condition_lock);
	cond_init(&con);
	sema = malloc(sizeof(struct semaphore));
	sema_init(sema, 1);
	remain_vehicle_count = thread_cnt;
	whole_vehicle_count = thread_cnt;
	movable_vehicle_count = thread_cnt;
}

void vehicle_loop(void *_vi)
{
	int res, i;
	int start, dest, step;
	step = 0;
	struct vehicle_info *vi = _vi;

	start = vi->start - 'A';
	dest = vi->dest - 'A';

	vi->position.row = vi->position.col = -1;
	vi->state = VEHICLE_STATUS_READY;
	vi->step = 0;

	// if (vi->id == 'a') {
	// 	while(1){}
	// }

	while (1) {
		/* vehicle main code */
		res = try_move(start, dest, vi);


		/*cs(모니터) */
		lock_acquire(&vehicle_condition_lock);

		//남은 카운트가 0보다 크면 wait하는 애 1개 더 있다고 알려주고 semaup, cond_wait
		if(remain_vehicle_count > 1){
			remain_vehicle_count--;

			cond_wait(&con, &vehicle_condition_lock);
		} else{
			//남은 카운트 0되면 모든 스레드가 움직였으니까 unitstepchanged
			unitstep_changed();
			cond_broadcast(&con, &vehicle_condition_lock);
			movable_vehicle_count += lock_vehicle_count;
			remain_vehicle_count = movable_vehicle_count;
			lock_vehicle_count = 0;
			//돌아가는 애 += lock 걸렸던 애들
			//remaincount = 돌아가는애 
			//기다리는애 0개로
		}

		lock_release(&vehicle_condition_lock);
		/* termination condition. */ 
		if (res == 0) {
			break;
		}

		/*cs */
		/* unitstep change! */

	}	

	/* status transition must happen before sema_up */
	vi->state = VEHICLE_STATUS_FINISHED;
}
