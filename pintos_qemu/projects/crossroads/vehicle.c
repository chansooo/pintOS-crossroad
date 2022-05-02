
#include <stdio.h>
#include <stdlib.h>

#include "threads/thread.h"
#include "threads/synch.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/map.h"
#include "projects/crossroads/ats.h"

struct intersectionTake
{
	int TakePath; //a, b, c, d에서 출발하는 애들 중 어떤 출발점이 선점할 건지
					//a=1, b=2, c=3, d=4
	int interTakeCount; //intersection안에 들어가있는 차가 몇개인지. 
						//0이 되면 얘 감싸는 세마포어 풀어주기
};

struct semaphore *csSema;
struct semaphore *intersectionSema; //
struct semaphore *abSema;
struct semaphore *bcSema;
struct semaphore *cdSema;
struct semaphore *daSema;

//struct semaphore *waitsignSema;
static struct intersectionTake intersectionTake1;


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






//옮길 좌표가 intersection 들어가기 직전인 네 곳이라면
//intersectiontake의 값들로 선점한 아이들 표시
static void is_position_enter_intersection(struct vehicle_info *vi){
	sema_down(intersectionSema);
	if(vi->start == 'A'){
		if(vi->dest == 'B'){
			lock_acquire(&vi->intersection_locks[2][4]);
		} else if(vi->dest == 'C'){
			lock_acquire(&vi->intersection_locks[2][4]);
			lock_acquire(&vi->intersection_locks[3][4]);
			lock_acquire(&vi->intersection_locks[4][4]);
		} else if(vi->dest == 'D'){
			lock_acquire(&vi->intersection_locks[2][4]);
			lock_acquire(&vi->intersection_locks[3][4]);
			lock_acquire(&vi->intersection_locks[4][4]);
			lock_acquire(&vi->intersection_locks[4][3]);
			lock_acquire(&vi->intersection_locks[4][2]);
		}
	} else if(vi->start == 'B'){
		if(vi->dest == 'A'){
			lock_acquire(&vi->intersection_locks[4][4]);
			lock_acquire(&vi->intersection_locks[4][3]);
			lock_acquire(&vi->intersection_locks[4][2]);
			lock_acquire(&vi->intersection_locks[3][2]);
			lock_acquire(&vi->intersection_locks[2][2]);
		} else if(vi->dest == 'C'){
			lock_acquire(&vi->intersection_locks[4][4]);
		} else if(vi->dest == 'D'){
			lock_acquire(&vi->intersection_locks[4][4]);
			lock_acquire(&vi->intersection_locks[3][4]);
			lock_acquire(&vi->intersection_locks[2][4]);
		}
	} else if(vi->start == 'C'){
		if(vi->dest == 'A'){
			lock_acquire(&vi->intersection_locks[4][2]);
			lock_acquire(&vi->intersection_locks[3][2]);
			lock_acquire(&vi->intersection_locks[2][2]);
		} else if(vi->dest == 'B'){
			lock_acquire(&vi->intersection_locks[4][2]);
			lock_acquire(&vi->intersection_locks[3][2]);
			lock_acquire(&vi->intersection_locks[2][2]);
			lock_acquire(&vi->intersection_locks[2][3]);
			lock_acquire(&vi->intersection_locks[2][4]);
		} else if(vi->dest == 'D'){
			lock_acquire(&vi->intersection_locks[4][2]);
		}
	} else if(vi->start =='D'){
		if(vi->dest == 'A'){
			lock_acquire(&vi->intersection_locks[2][2]);
		} else if(vi->dest == 'B'){
			lock_acquire(&vi->intersection_locks[2][2]);
			lock_acquire(&vi->intersection_locks[3][2]);
			lock_acquire(&vi->intersection_locks[4][2]);
		} else if(vi->dest == 'C'){
			lock_acquire(&vi->intersection_locks[2][2]);
			lock_acquire(&vi->intersection_locks[2][4]);
			lock_acquire(&vi->intersection_locks[2][4]);
			lock_acquire(&vi->intersection_locks[2][4]);
			lock_acquire(&vi->intersection_locks[2][4]);
		}
	}
	sema_up(intersectionSema);
}
//intersection 빠져나갈 때 count--해주고 
static void is_position_out_intersection(struct vehicle_info *vi){
		sema_down(intersectionSema);
	if(vi->start == 'A'){
		if(vi->dest == 'B'){
			lock_acquire(&vi->intersection_locks[2][4]);
		} else if(vi->dest == 'C'){
			lock_release(&vi->intersection_locks[2][4]);
			lock_release(&vi->intersection_locks[3][4]);
			lock_release(&vi->intersection_locks[4][4]);
		} else if(vi->dest == 'D'){
			lock_release(&vi->intersection_locks[2][4]);
			lock_release(&vi->intersection_locks[3][4]);
			lock_release(&vi->intersection_locks[4][4]);
			lock_release(&vi->intersection_locks[4][3]);
			lock_release(&vi->intersection_locks[4][2]);
		}
	} else if(vi->start == 'B'){
		if(vi->dest == 'A'){
			lock_release(&vi->intersection_locks[4][4]);
			lock_release(&vi->intersection_locks[4][3]);
			lock_release(&vi->intersection_locks[4][2]);
			lock_release(&vi->intersection_locks[3][2]);
			lock_release(&vi->intersection_locks[2][2]);
		} else if(vi->dest == 'C'){
			lock_release(&vi->intersection_locks[4][4]);
		} else if(vi->dest == 'D'){
			lock_release(&vi->intersection_locks[4][4]);
			lock_release(&vi->intersection_locks[3][4]);
			lock_release(&vi->intersection_locks[2][4]);
		}
	} else if(vi->start == 'C'){
		if(vi->dest == 'A'){
			lock_release(&vi->intersection_locks[4][2]);
			lock_release(&vi->intersection_locks[3][2]);
			lock_release(&vi->intersection_locks[2][2]);
		} else if(vi->dest == 'B'){
			lock_release(&vi->intersection_locks[4][2]);
			lock_release(&vi->intersection_locks[3][2]);
			lock_release(&vi->intersection_locks[2][2]);
			lock_release(&vi->intersection_locks[2][3]);
			lock_release(&vi->intersection_locks[2][4]);
		} else if(vi->dest == 'D'){
			lock_release(&vi->intersection_locks[4][2]);
		}
	} else if(vi->start =='D'){
		if(vi->dest == 'A'){
			lock_release(&vi->intersection_locks[2][2]);
		} else if(vi->dest == 'B'){
			lock_release(&vi->intersection_locks[2][2]);
			lock_release(&vi->intersection_locks[3][2]);
			lock_release(&vi->intersection_locks[4][2]);
		} else if(vi->dest == 'C'){
			lock_release(&vi->intersection_locks[2][2]);
			lock_release(&vi->intersection_locks[2][4]);
			lock_release(&vi->intersection_locks[2][4]);
			lock_release(&vi->intersection_locks[2][4]);
			lock_release(&vi->intersection_locks[2][4]);
		}
	}
	sema_up(intersectionSema);
}

static int is_position_outside(struct position pos)
{
	return (pos.row == -1 || pos.col == -1);
}

/* return 0:termination, 1:success, -1:fail */
// vehicle 이동하는 함수
static int try_move(int start, int dest, int step, struct vehicle_info *vi)
{
	//현재 위치, 다음 위치 받아옴
	struct position pos_cur, pos_next;

	pos_next = vehicle_path[start][dest][step];
	pos_cur = vi->position;

	

	// running인데 맵 바깥으로 나가면 없애줌
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
	
	/* lock next position */
	//다음으로 갈 애 잠궈버려서 못 가도록
	
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

	is_position_enter_intersection(&vi);

	is_position_out_intersection(&vi);

	
	return 1;
}


void init_on_mainthread(int thread_cnt){
	/* Called once before spawning threads */
	csSema = malloc(sizeof (struct semaphore));
	intersectionSema = malloc(sizeof (struct semaphore));
	daSema = malloc(sizeof (struct semaphore));
	abSema = malloc(sizeof (struct semaphore));
	bcSema = malloc(sizeof (struct semaphore));
	cdSema = malloc(sizeof (struct semaphore));
	sema_init(csSema, 1);
	sema_init(intersectionSema, 1);
	sema_init(daSema, 1);
	sema_init(abSema, 1);
	sema_init(bcSema, 1);
	sema_init(cdSema, 1);
	intersectionTake1.interTakeCount =0;
	intersectionTake1.TakePath =0;
}

void vehicle_loop(void *_vi)
{
	int res;
	int start, dest, step;

	struct vehicle_info *vi = _vi;

	start = vi->start - 'A';
	dest = vi->dest - 'A';

	vi->position.row = vi->position.col = -1;
	vi->state = VEHICLE_STATUS_READY;

	step = 0;
	while (1) {
		/* vehicle main code */
		res = try_move(start, dest, step, vi);

		if (res == 1) {
			step++;
		}

		/* termination condition. */ 
		if (res == 0) {
			break;
		}

		/* unitstep change! */
		unitstep_changed();
		//
		crossroads_step++;
	}	

	/* status transition must happen before sema_up */
	vi->state = VEHICLE_STATUS_FINISHED;
}