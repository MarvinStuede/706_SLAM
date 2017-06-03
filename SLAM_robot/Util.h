#ifndef UTIL_H_
#define UTIL_H_
enum statesMain{
	NONE,
	STATE_INIT,
	STATE_TURN,
	STATE_DRIVE_WALL,
	STATE_WAIT,
  STATE_OBSTACLE
};

enum statesInit{
	INIT_SPIN,
	INIT_APP_WALL_1,
	INIT_APP_WALL_2,
	INIT_APP_WALL_3,
	INIT_APP_WALL_4
};

enum statesTurn{
	TURN_GYRO,
	TURN_WALL,
};
#endif
