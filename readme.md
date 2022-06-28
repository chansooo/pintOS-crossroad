# Thread Systems and Synchronization

본 레포지토리에서는 pintOS 내부에서 단위스텝 문제, 교차로 동기화 문제를 해결해볼 것이다.

### 목표

- 왕복 2차선의 동서남북으로 길이 있는 교차로에서 차량이 들어올 때 차량들이 `Deadlock(교착상태)` 에 빠지지 않도록 구현한다.
- 차들은 1틱당 1번씩 움직이게 되는데 움질일 수 있는 모든 차량이 움직이고 난 뒤 틱을 종료하고 단위 스텝을 1씩 올리도록 한다.

`pintos_qemu->projects->crossroads->vehicle.c` 의 코드를 수정하여 진행하였다.

## 단위스텝 해결

모니터를 구현하여 사용해서 단위스텝을 구현

임계영역의 상호배제를 보장하기 위해 임계영역과 관련된 차량 스레드들을 모두 재우고, 오직 하나의 스레드만 임계영역에 접근할 수 있도록 하고, 모든 스레드가 끝날 경우 스레드들을 broadcast로 깨워줘서 스텝이 끝난 것을 알려주도록 구현하였다.

```c
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
```

한 단위스텝이 끝나고 다음 단위스텝에서 몇 개의 차량 스레드를 관리해야하는지를 알기 위해 threadcnt 변수, waitcnt 변수를 선언하여 사용했다.

Lock을 이용하여 lock_acquire와 lock_release를 vehicle_loop 함수의 while문 내부에 구현을 하고 다른 스레드의 간섭을 방지하지 위해 lock_acquire과 lock_release 사이에서 모든 차량 스레드가 끝났는지를 확인하였다.
try_move 함수 동작을 마치고 마지막 남은 차량 스레드가 아닌 다른 스레드들은 waitcnt를 1 올려준 후  cond_wait(&waiters, &wait_lock)을 통해 wait하도록 구현했고, 만약 wait가 풀린다면 waitcnt를 다시 1 내려주도록 구현했다.
마지막 차량 스레드가 try_move함수 동작을 마치면 waitcnt가 threadcnt보다 1 큰 상황이 되는데, 이때 한 단위스텝이끝났다는것을인지할수있고, unitstep_changed()를 실행해서 단위 스텝을 올려주고 cond_broadcast로 wait하고 있던 모든 스레드들을 깨워준다.

만약 이후 1.2에서 설명하는 바와 같이 try_move 함수의 return 값이 0이라면 동작할 수 없는 차량 스레드이므로 threadcnt를 1 줄여준다.
마지막으로 반복문을 반복하기 전에(새로운 단위 스텝을 진행하기 전에) 현재 점유 중인 스레드가 점유를 양보하고 ready_list에 스레드를 추가할 때 사용하는 함수인 thread_yield()함수를 호출하여 모든 스레드들이 완전히 단위 스텝을 끝낼 수 있도록 유도한다.

## 교차로 동기화 해결

차량 동기화의 문제들로는 교차로에 진입한 차들이 서로가 선점한 칸을 기다리는 교착상태, 하나의 출발지점에서 시작한 차들이 계속 줄지어서 들어와 다른 시작점을 가진 차들이 계속 기다리는 기아 상태등이 있다.
이러한 동기화 문제를 해결하기 위해서 3x3 교차로 내부의 각각의 칸을 Critical Section으로 지정하고 교차로에 진입하기 전에 각 차의 예상 경로가 선점되었는지를 확인하고 모두 이용 가능하다면 교차로로 진입하고, 선점된 칸이 하나라도 있을 경우 block 처리한다.

```c
static int is_position_crossroad(struct position pos)
{
	return (2 <= pos.row && pos.row <= 4) && (2 <= pos.col && pos.col <= 4);
}
```

position이 교차로 내부인지 확인하는 함수이다. Row와 column이 둘 다 2,3,4 중 하나라면 교차로 내부에 있으므로 1을 반환하고 아니라면 0을 반환한다.

```c
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
```

교차로에 진입할 때 차량 스레드가 지나가는 칸을 선점하고, 만약 지나가고자 하는 칸이 선점되어 있다면 block되도록 구현.
다음 스텝이 교차로 내부인지 외부인지 판별하고 교차로를 벗어날 때까지 재귀적으로 구현하였다.

if문 내부에 재귀적으로 함수를 넣어서 재귀하는 함수 하나가 0을 반환하면 나머지도 0을 반환하게 해서 차량 스레드가 선점해야할 교차로 내부의 좌표를 전부 선점하거나, 전부 그대로 두도록 하였다.

```c
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
```

교차로에서 빠져나올 때 차가 선점했던 교차로 내부의 칸을 release 해준다. 현재교차로내부라면현재좌표의선점이해당차량 스레드의 것인지를 확인하고 맞다면 현재 좌표의 선점을 푼다. 그리고 step을 1을 줄여 교차로를 벗어날 때까지전까지재귀적으로그전의지나온교차로 내부의 좌표 또한 release한다.

### try move 메소드

1)교차로에 진입하려고 할 경우

```c
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
	}
```

사전에 만들어 둔 try_lock_crossroad 함수를 호출하여 차량 스레드가 교차로에서 지나가야할 길이 모두 선점당하지 않았다면, 현재 좌표의 map_lock을 release하고 교차로로 진입한다.

교차로에서 지나가야할 칸 중 하나라도 선점당해있다면 대기한다.

2)교차로 내부에서 움직이는 경우:

```c
else if (is_position_crossroad(pos_cur) && is_position_crossroad(pos_next)) { /* crossroad to crossroad (do not release lock) */
		vi->position = pos_next;
		/* check moved */
		vi->step++;
		return 1;
	}
```

이미 교차로 내부에서 지나갈 경로를 lock시켜놨기 때문에 다음 좌표로 position을 옮겨주고 step을 올려준다

3)교차로에서 나오려고 하는 경우:

```c
else if (is_position_crossroad(pos_cur) && !is_position_crossroad(pos_next)) { /* about to out crossroad */
		/* lock next */
		lock_acquire(&vi->map_locks[pos_next.row][pos_next.col]);
		/* release crossroad path */
		release_lock_crossroad(pos_cur, start, dest, vi->step-1, vi);
		/* proceed */
		vi->position = pos_next;
		/* check moved */
		vi->step++;
		return 1;
	}
```

다음 가야할 좌표를 선점하고, 교차로 내부의 선점한 경로들을 풀어준다. 그 후 현재 좌표를 풀어준 후 좌표를 다음 가야할 좌표로 바꿔준다.

4)교차로가 아닌 곳에서 아닌 곳으로 갈 경우:

```c
else {
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
```

다음에 진행할 좌표가 선점되었는지 확인하고 선점되지 않았다면 선점하고 현재 좌표를 풀어준다. 이때 status가 ready라면 running으로 변경시켜준다. 정상적으로 진행됐다면 return 값으로 1을 반환하고, 끝나거나, ready 상태인 경우 0을  return해주고, 앞의 스레드보다 먼저 실행되어 선점을 해야하는데 못 한 경우 2를 return해준다