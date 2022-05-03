
#include <stdio.h>

#include "threads/thread.h"
#include "threads/synch.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/map.h"
#include "projects/crossroads/ats.h"

<<<<<<< HEAD
=======
struct intersectionTake
{
	int TakePath; //a, b, c, d에서 출발하는 애들 중 어떤 출발점이 선점할 건지
					//a=1, b=2, c=3, d=4
	int interTakeCount; //intersection안에 들어가있는 차가 몇개인지. 
						//0이 되면 얘 감싸는 세마포어 풀어주기
};

struct semaphore *csSema;
struct semaphore *intersectionSema; //
struct semaphore *moveSema;
//struct semaphore *waitsignSema;
static struct intersectionTake intersectionTake1;
>>>>>>> main


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

<<<<<<< HEAD
=======


//옮길 좌표가 intersection 들어가기 직전인 네 곳이라면
//intersectiontake의 값들로 선점한 아이들 표시
static void is_position_enter_intersection(struct position a){

	if((a.col == 1 && a.row == 4)||(a.col == 4 && a.row == 5)||(a.col == 5 && a.row == 2)||(a.col == 2 && a.row == 1)){
		sema_down(csSema);
		if(intersectionTake1.interTakeCount <= 0){ //교차로에 차 없으면
			if(a.col == 1 && a.row == 4){ //A에서 교차로 진입
				sema_down(intersectionSema);
				intersectionTake1.TakePath = 1;
				intersectionTake1.interTakeCount++;
			} else if(a.col == 4 && a.row == 5){ //B에서 교차로 진입
				sema_down(intersectionSema);
				intersectionTake1.TakePath = 2;
				intersectionTake1.interTakeCount++;
			} else if(a.col == 5 && a.row == 2){ //C에서 교차로 진입
				sema_down(intersectionSema);
				intersectionTake1.TakePath = 3;
				intersectionTake1.interTakeCount++;
			} else if(a.col == 2 && a.row == 1){ // D에서 교차로 진입
				sema_down(intersectionSema);
				intersectionTake1.TakePath = 4;
				intersectionTake1.interTakeCount++;
			}
			sema_up(csSema);
		} else{ //교차로에 차 있으면 있는 차 어떤 출발점인지 보고 같은 출발점이면 출발, 아니면 semadown
			//A 출발 차가 교차로 먹고 들어오는 차도 A에서 출발일 때
			if(intersectionTake1.TakePath == 1 && a.col == 1 && a.row == 4){
				intersectionTake1.interTakeCount++;
				sema_up(csSema);
			}else if(intersectionTake1.TakePath == 2 && a.col == 4 && a.row == 5){
				intersectionTake1.interTakeCount++;
				sema_up(csSema);
			}else if(intersectionTake1.TakePath == 3 && a.col == 5 && a.row == 2){
				intersectionTake1.interTakeCount++;
				sema_up(csSema);
			} else if(intersectionTake1.TakePath == 4 && a.col == 2 && a.row == 1){
				intersectionTake1.interTakeCount++;
				sema_up(csSema);	
			} else{ //같은 출발지점이 아닐 때
				sema_up(csSema);
				sema_down(intersectionSema);
				//intersectionSema 선점했으면 takePath 값 바꿔주기
				if(a.col == 1 && a.row == 4){
					intersectionTake1.TakePath = 1;
				} else if(a.col == 4 && a.row == 5){
					intersectionTake1.TakePath = 2;
				}else if(a.col == 5 && a.row == 2){
					intersectionTake1.TakePath = 3;
				}else if(a.col == 2 && a.row == 1){
					intersectionTake1.TakePath = 4;
				}
			}
		
		}
	}
	
}
//intersection 빠져나갈 때 count--해주고 
static void is_position_out_intersection(struct position a){
	if((a.col == 1 && a.row == 2)||(a.col == 2 && a.row == 5)||(a.col == 5 && a.row == 4)||(a.col == 4 && a.row == 1)){
		sema_down(csSema);
		//차가 교차로 빠져나갈려고 하면 count 1씩 빼줌
		if(a.col == 1 && a.row == 2){ //A로 들어가는 차
			intersectionTake1.interTakeCount--;
		} else if(a.col == 2 && a.row == 5){  //B로 들어가는 차
			intersectionTake1.interTakeCount--;
		}else if(a.col == 5 && a.row == 4){  //C로 들어가는 차
			intersectionTake1.interTakeCount--;
		}else if(a.col == 4 && a.row == 1){  //D로 들어가는 차
			intersectionTake1.interTakeCount--;
		}

		if(intersectionTake1.interTakeCount == 0){
			sema_up(intersectionSema);
		}
		sema_up(csSema);
	}
	
}

>>>>>>> main
static int is_position_outside(struct position pos)
{
	return (pos.row == -1 || pos.col == -1);
}

static int is_position_crossroad(struct position pos)
{
	return (2 <= pos.row && pos.row <= 4) && (2 <= pos.col && pos.col <= 4);
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

<<<<<<< HEAD
=======
	

	// running인데 맵 바깥으로 나가면 없애줌
>>>>>>> main
	if (vi->state == VEHICLE_STATUS_RUNNING) {
		/* check termination */
		if (is_position_outside(pos_next)) {
			/* actual move */
			vi->position.row = vi->position.col = -1;
			/* release previous */
			lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
			return 0;
		}
	}
<<<<<<< HEAD

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
=======
	
	/* lock next position */
	//다음으로 갈 애 잠궈버려서 못 가도록
	
>>>>>>> main
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
<<<<<<< HEAD
	/* check moved */
	vi->step++;
	
end_turn: 
=======
//	sema_down(moveSema);
	is_position_enter_intersection(vi->position);

	is_position_out_intersection(vi->position);
//	sema_up(moveSema);
	
>>>>>>> main
	return 1;
}

void init_on_mainthread(int thread_cnt){
	/* Called once before spawning threads */
<<<<<<< HEAD
=======
	csSema = malloc(sizeof (struct semaphore));
	intersectionSema = malloc(sizeof (struct semaphore));
	moveSema = malloc(sizeof (struct semaphore));
	sema_init(csSema, 1);
	sema_init(intersectionSema, 1);
	sema_init(moveSema, 1);
	intersectionTake1.interTakeCount =0;
	intersectionTake1.TakePath =0;
>>>>>>> main
}

void vehicle_loop(void *_vi)
{
	int res, i;
	int start, dest, step;

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

		/* termination condition. */ 
		if (res == 0) {
			break;
		}

		/* unitstep change! */
		unitstep_changed();
	}	

	/* status transition must happen before sema_up */
	vi->state = VEHICLE_STATUS_FINISHED;
}
