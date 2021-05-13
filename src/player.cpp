#include "AI.h"
#include "Constants.h"

//为假则play()调用期间游戏状态更新阻塞，为真则只保证当前游戏状态不会被状态更新函数与GameApi的方法同时访问
extern const bool asynchronous = false;

#include <ext/pb_ds/priority_queue.hpp>
#include <random>
#include <iostream>
#include <unordered_map>
#include <unistd.h>
#include <unordered_set>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <cstring>
#include <memory>
#include <list>
// #include "MainWindow.h"


/****************************************/
/*                                      */
/*          Struct & Enum               */
/*                                      */
/****************************************/

namespace
{
	[[maybe_unused]] std::uniform_real_distribution<double> direction(0, 2 * 3.1415926);
	[[maybe_unused]] std::default_random_engine e{ std::random_device{}() };
}

typedef std::array<int, 2> Position;

struct frontier_node
{
	double distance;
	std::array<int, 2> position;
	bool operator>(const frontier_node& n) const
	{
		return distance > n.distance;
	}
};

const int bulletBomb[7] = {3, 3, 7, 1, 7, 3, 3};
struct bulletCache
{
	uint16_t teamID;
	double facingDirection;
	double moveSpeed;
	double avoidRad;
	double tmpx;
	double tmpy;
	int type;
	bulletCache(uint16_t i, double f, double m, double avoidRad, double tmpx, double tmpy, int type) : 
	teamID(i), facingDirection(f), moveSpeed(m), avoidRad(avoidRad), tmpx(tmpx), tmpy(tmpy), type(type){}
	bool operator<(const bulletCache& a) const
	{
    	return moveSpeed < a.moveSpeed;
	}
};

enum State {WAIT_BULLET, EXPAND_FIELD, SEARCH_ENEMY};
enum Action {MOVE, ATTACK, PICK, USE, WAIT};
enum Job {PURPLE_FISH, EGG_MAN, MONKEY_DOCTOR};

/****************************************/
/*                                      */
/*          Global Variables             */
/*                                      */
/****************************************/

extern const THUAI4::JobType playerJob = THUAI4::JobType::Job3; // Purple Fish
// extern const THUAI4::JobType playerJob = THUAI4::JobType::Job4; // Monkey 
// extern const THUAI4::JobType playerJob = THUAI4::JobType::Job5; // Egg Mandoctor

const int ZOOM = 10;
const int LENGTH = 50;
const static unsigned char defaultMap[LENGTH][LENGTH] = {
	//0                             10                            20                            30                            40                         49
	{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, //0
	{1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1},
	{1, 1, 1, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 1, 1, 1},
	{1, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 1, 1},
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},

	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}, //10
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1},
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},

	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}, //20
	{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},

	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}, //30
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1},
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},

	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}, //40
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 1, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 1, 1},
	{1, 1, 1, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 1, 1, 1},
	{1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1},
	{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1} //49
};

static bool canStepUnColored = true;
static int unColoredDistance = 10;
const static double MAX_DISTANCE = 9999999999;
unsigned char dynamicMap[LENGTH][LENGTH];

std::unordered_map<int64_t, THUAI4::Character> players;
std::unordered_map<int64_t, THUAI4::Prop> props;

static GameApi* gameInfo;
static Action lastAction;
static Job job;

const unsigned char DONT_MOVE = 8;
static unsigned char route[LENGTH][LENGTH];
static double distance_table[LENGTH][LENGTH];
static char colorMap[LENGTH][LENGTH];
static char colorValueMap[LENGTH][LENGTH];
static double areaValue[4];
static int frame = 0, tiredFrame = 0;
static int attackTime = 0, attackHp = 0;
static int nowBulletNum = 0;
static int lastX, lastY, lastAttackAngle;
static bool isAct, getItem;
static clock_t processBegin, processEnd;

Position lastPosition, nowPosition, nextPosition, nowTarget, finalTarget;
static std::vector<std::vector<Position>> final_target_list_choice;
std::vector<Position> final_target_list;
std::priority_queue<bulletCache> bulletNow;

const std::array<int, 2> operate[] = {
	{1, 0},
	{1, 1},
	{0, 1},
	{-1, 1},
	{-1, 0},
	{-1, -1},
	{0, -1},
	{1, -1} };

const int dirX[8]= {1, 1, 0, -1, -1, -1, 0, 1};
const int dirY[8] = {0, 1, 1, 1, 0, -1, -1, -1};

/****************************************/
/*                                      */
/*        Overload Functions            */
/*                                      */
/****************************************/

std::array<int, 2> operator+(const std::array<int, 2>& p1, const std::array<int, 2>& p2)
{
	return std::array<int, 2>{p1[0] + p2[0], p1[1] + p2[1]};
}
std::array<int, 2> operator+(const std::array<int, 2> &p1, int i)
{
	return std::array<int, 2>{p1[0] + i, p1[1] + i};
}
std::array<int, 2> operator-(const std::array<int, 2>& p1, const std::array<int, 2>& p2)
{
	return std::array<int, 2>{p1[0] - p2[0], p1[1] - p2[1]};
}
std::array<int, 2> operator-(const std::array<int, 2>& p1)
{
	return std::array<int, 2>{-p1[0], -p1[1]};
}
std::array<int, 2> operator*(const std::array<int, 2> &p1, int d)
{
	return std::array<int, 2>{p1[0] * d, p1[1] * d};
}
std::array<int, 2> operator/(const std::array<int, 2> &p1, int d)
{
	return std::array<int, 2>{p1[0] / d, p1[1] / d};
}

double pointToLineDistance(const std::array<int, 2UL> &point, const std::array<int, 2UL> &linePoint1, const std::array<int, 2UL> &linePoint2)
{
	double A = linePoint2[1] - linePoint1[1];
	double B = linePoint1[0] - linePoint2[0];
	double C = linePoint2[0] * linePoint1[1] - linePoint1[0] * linePoint2[1];
	return fabs(A * point[0] + B * point[1] + C) / sqrt(A * A + B * B);
}

bool operator==(const std::array<int, 2UL>& p1, const std::array<int, 2UL>& p2)
{
	return p1[0] == p2[0] && p1[1] == p2[1];
}

/****************************************/
/*                                      */
/*          Math Functions              */
/*                                      */
/****************************************/

inline double length(const std::array<int, 2>& p1)
{
	return sqrt(p1[0] * p1[0] + p1[1] * p1[1]);
}

inline double getMoveAngle(std::list<unsigned char>::iterator it)
{
	return *it * M_PI / 4;
}

inline double getPointToPointAngle(double sourceX, double sourceY, double targetX, double targetY)
{
    double angleR = atan2(targetY - sourceY, targetX - sourceX);
    if (angleR < 0) angleR += 2 * M_PI;
    return angleR;
}

inline double getPointToPointDistance(double sourceX, double sourceY, double targetX, double targetY)
{
    return sqrt((sourceX - targetX) * (sourceX - targetX) + (sourceY - targetY) * (sourceY - targetY));
}

inline double getGridDistance(Position source, Position target)
{
    return sqrt((source[0] - target[0]) * (source[0] - target[0]) + (source[1] - target[1]) * (source[1] - target[1]));
}

inline double angleToRadian(double value)
{
	return value * M_PI / 180.;
}

inline double radianToAngle(double value)
{
	return value * 180. / M_PI;
}

inline int CordToGrid(double value)
{   
    return int(value / 1000.);
}

inline int GridToCord(int value)
{
    return value * 1000 + 500;
}

/****************************************/
/*                                      */
/*          Normal Function             */
/*                                      */
/****************************************/

void dijkstra(const std::array<int, 2> &point)
{
	bool dont_search[LENGTH][LENGTH] = {{0}};
	memset(route, DONT_MOVE, sizeof(route));
	memset(colorValueMap, 0, sizeof(colorValueMap));
	// std::cout << sizeof(route) << std::endl;
	__gnu_pbds::priority_queue<frontier_node, std::greater<frontier_node>> frontier;
	for (int i = 0; i<50; ++i)
		for(int j = 0; j<50; ++j)
			distance_table[i][j] = MAX_DISTANCE;
	std::array<int, 2> int_part = {int(point[0] / 1000), int(point[1] / 1000)};
	std::array<int, 2> decimal_part = {point[0] % 1000, point[1] % 1000};

	dont_search[int_part[0]][int_part[1]] = 1;
	// frontier.push(frontier_node{0, int_part});
	distance_table[int_part[0]][int_part[1]] = 0;
	colorValueMap[int_part[0]][int_part[1]] = colorMap[int_part[0]][int_part[1]];

	bool avaiable[8];
	{
		std::array<int, 2UL> p[8];
		for (int i = 0; i < 8; ++i)
			p[i] = int_part + operate[i];
		memset(avaiable, 1, sizeof(avaiable));
		for (int i = 0; i < 8; i++)
		{
			if (dynamicMap[p[i][0]][p[i][1]])
			{
				// std::cout << "setting unavaiable : " << i << std::endl;
				avaiable[i] = 0;
				if (!(i % 2))
				{
					avaiable[(i + 8 - 1) % 8] = 0;
					avaiable[(i + 1) % 8] = 0;
				}
			}
		}
		if (decimal_part[1] < 500)
		{
			if (dynamicMap[p[5][0]][p[5][1]])
				avaiable[4] = 0;
			if (dynamicMap[p[7][0]][p[7][1]])
				avaiable[0] = 0;
		}
		else if (decimal_part[1] > 500)
		{
			if (dynamicMap[p[3][0]][p[3][1]])
				avaiable[4] = 0;
			if (dynamicMap[p[1][0]][p[1][1]])
				avaiable[0] = 0;
		}
		if (decimal_part[0] < 500)
		{
			if (dynamicMap[p[3][0]][p[3][1]])
				avaiable[2] = 0;
			if (dynamicMap[p[5][0]][p[5][1]])
				avaiable[6] = 0;
		}
		else if (decimal_part[0] > 500)
		{
			if (dynamicMap[p[1][0]][p[1][1]])
				avaiable[2] = 0;
			if (dynamicMap[p[7][0]][p[7][1]])
				avaiable[6] = 0;
		}
		if (
			(dynamicMap[p[1][0]][p[1][1]] && pointToLineDistance(operate[1] * 500 + 500, decimal_part, decimal_part + (operate[3] * 500)) < 600) ||
			(dynamicMap[p[5][0]][p[5][1]] && pointToLineDistance(operate[5] * 500 + 500, decimal_part, decimal_part + (operate[3] * 500)) < 600))
			avaiable[3] = avaiable[7] = 0;
		if (
			(dynamicMap[p[3][0]][p[3][1]] && pointToLineDistance(operate[3] * 500 + 500, decimal_part, decimal_part + (operate[1] * 500)) < 600) ||
			(dynamicMap[p[7][0]][p[7][1]] && pointToLineDistance(operate[7] * 500 + 500, decimal_part, decimal_part + (operate[1] * 500)) < 600))
			avaiable[1] = avaiable[5] = 0;
		for (int i = 0; i < 8; ++i)
		{
			if (!avaiable[i])
				continue;
			auto p = int_part + operate[i];
			double distance = i % 2 ? sqrt(2) : 1;
			frontier.push(frontier_node{distance, p});
            if (colorMap[p[0]][p[1]] != 1) distance_table[p[0]][p[1]] = distance * unColoredDistance;
			else distance_table[p[0]][p[1]] = distance;
			colorValueMap[p[0]][p[1]] = colorMap[p[0]][p[1]];
			route[p[0]][p[1]] = i;
		}
	}
	while (1)
	{
		if (frontier.empty())
		{
			// std::cout << "frontier is empty" << std::endl;
			break;
		}
		auto searching_point = frontier.top().position;
		frontier.pop();
		// std::cout << "searching :  (" << searching_point[0] << "," << searching_point[1] << ")" << std::endl;
		auto distance = distance_table[searching_point[0]][searching_point[1]];
		auto color = colorValueMap[searching_point[0]][searching_point[1]];
		// std::cout << "distance :  " << distance << std::endl;
		dont_search[searching_point[0]][searching_point[1]] = 1;
		memset(avaiable, 1, sizeof(avaiable));
		for (int i = 0; i < 8; i ++)
		{
			auto p = searching_point + operate[i];
			if (dynamicMap[p[0]][p[1]])
			{
				// std::cout << "setting unavaiable : " << i << std::endl;
				avaiable[i] = 0;
				if (!(i % 2))
				{
					avaiable[(i + 8 - 1) % 8] = 0;
					avaiable[(i + 1) % 8] = 0;
				}
			}
		}

		for (int i = 0; i < 8; ++i)
		{
			if (!avaiable[i])
				continue;
			auto p = searching_point + operate[i];
			// std::cout << "updating children :  (" << p[0] << "," << p[1] << ")" << std::endl;
			double neighbor_distance = i % 2 ? sqrt(2) : 1;
			if (colorMap[p[0]][p[1]] != 1)
				neighbor_distance *= unColoredDistance;
			if (dont_search[p[0]][p[1]])
				continue;
			if (distance_table[p[0]][p[1]] < MAX_DISTANCE && distance + neighbor_distance > distance_table[p[0]][p[1]])
				continue;
			auto iter = frontier.begin();
			for (; iter != frontier.end(); iter++)
			{
				if (iter->position == p)
					break;
			}
			// std::cout << "updating distance : " << distance + neighbor_distance << std::endl;
			if (iter == frontier.end())
				frontier.push(frontier_node{distance + neighbor_distance, p});
			else
				frontier.modify(iter, frontier_node{distance + neighbor_distance, p});
			distance_table[p[0]][p[1]] = distance + neighbor_distance;
			colorValueMap[p[0]][p[1]] = color + colorMap[p[0]][p[1]];
			route[p[0]][p[1]] = i;
			// std::cout << "complete a child  " << std::endl;
		}
	}
}

Position findNearestAvaiablePoint(Position p)
{
	for (int i = 0 ; i < 10 ; ++i)
	{
		std::cout << "i : " << i << std::endl;
		for (int x = std::max(p[0] - i, 0) ; x <= std::min(p[0] + i, 49) ; ++x)
		{
			// std::cout << "x : " << x << std::endl;
			for (int y : {std::max(p[1] - i, 0), std::min(p[1] + i, 49)})
			{
				// std::cout << "y : " << y << std::endl;
				if(route[x][y] != DONT_MOVE)
				{
					return {x, y};
				}
			}
		}
		for (int y = std::max(p[1] - i + 1, 0) ; y <= std::min(p[1] + i - 1, 49) ; ++y)
		{
			// std::cout << "y : " << y << std::endl;
			for (int x : {std::max(p[0] - i, 0), std::min(p[0] + i, 49)})
			{
				// std::cout << "x : " << x << std::endl;
				if(route[x][y] != DONT_MOVE)
				{
					return {x, y};
				}
			}
		}
	}
	return p;
}

std::list<unsigned char> searchWayFromMap(
	std::array<int, 2> start, std::array<int, 2> end,
	std::function<void(std::array<int, 2>)> func = [](std::array<int, 2> p) {})
{
	// std::cout << "start : " << start[0] << "," << start[1] <<std::endl;
	// std::cout << "end : " << end[0] << "," << end[1] <<std::endl;
	auto new_end = findNearestAvaiablePoint(end);
	// std::cout << "new end : " << new_end[0] << "," << new_end[1] <<std::endl;
	std::list<unsigned char> result;
	while (1)
	{
		func(new_end);
		result.push_front(route[new_end[0]][new_end[1]]);
		if (new_end == start)
			break;
        if (route[new_end[0]][new_end[1]] == DONT_MOVE)
            break;
		// std::cout << new_end[0] << "  " << new_end[1] << "  " << (int)route[new_end[0]][new_end[1]] << std::endl;
		Position old_end = new_end;
		new_end = old_end - operate[route[old_end[0]][old_end[1]]];
		if (new_end == start)
		{
			if(!canStepUnColored && (colorMap[new_end[0]][new_end[1]] == 1) && (colorMap[old_end[0]][old_end[1]] != 1))
				result.push_front(DONT_MOVE);
			break;
		}
		// std::cout<<"eee"<<std::endl;
	}
	return result;
}

void initialization(GameApi& g)
{
    auto self = g.GetSelfInfo();
    
    if (playerJob == THUAI4::JobType::Job3) job = PURPLE_FISH;
    else if (playerJob == THUAI4::JobType::Job4) job = MONKEY_DOCTOR;
    else if (playerJob == THUAI4::JobType::Job5) job = EGG_MAN;

    final_target_list_choice =
    {
        // Route 0 for Purple Fish(LeftUpper Corner)
        {{6, 6}, {16, 6}, {25, 15}, {34, 6}, {44, 6}, {44, 16}, 
            {44, 26}, {44, 35}, {44, 44}, {41, 44}, {30, 44}, {20, 44}, 
            {9, 44}, {6, 44}, {6, 34}, {6, 24}, {6, 14}},
        // Route 1 for Purple fish(LeftUpper Corner)
        {{6, 14}, {6, 24}, {6, 34}, {6, 44}, {9, 44}, {20, 44}, 
            {30, 44}, {41, 44}, {44, 44}, {44, 35}, {44, 26}, {44, 16}, 
            {44, 6}, {34, 6}, {25, 15}, {16, 6}, {6, 6}},
        // Route 2 for Purple fish(LeftBottom Corner)
        {{44, 6}, {34, 6}, {25, 15}, {16, 6}, {6, 6},
            {6, 14}, {6, 24}, {6, 34}, {6, 44}, {9, 44}, {20, 44}, 
            {30, 44}, {41, 44}, {44, 44}, {44, 35}, {44, 26}, {44, 16}},
        // Route 3 for Purple Fish(LeftBottom Corner)
        {{44, 16}, {44, 26}, {44, 35}, {44, 44}, {41, 44}, {30, 44},
            {20, 44}, {9, 44}, {6, 44}, {6, 34}, {6, 24}, {6, 14},
            {6, 6}, {16, 6}, {25, 15}, {34, 6}, {44, 6}},
        // Route 4 for Purple Fish
        {{16, 15}, {16, 24}, {16, 34}, {26, 34}, {36, 34}, {36, 25}, {36, 15}, {25, 25}},
        // Route 5 for Monkey Docter & Egg Man
        {{25, 25}, {36, 15}, {36, 25}, {36, 34}, {26, 34}, {16, 34}, {16, 24}, {16, 15}}
    };

    nowPosition = { CordToGrid(self->x), CordToGrid(self->y) };
    if (job == MONKEY_DOCTOR || job == EGG_MAN)  final_target_list = final_target_list_choice[5];
    else if (job == PURPLE_FISH)
    {
        if (nowPosition[1] < 25)
        {
            if (nowPosition[0] < 25)
            {
                if (nowPosition[1] == 2) final_target_list = final_target_list_choice[0];
                if (nowPosition[1] == 3) final_target_list = final_target_list_choice[1];
            }
            if (nowPosition[0] > 25)
            {
                if (nowPosition[1] == 2) final_target_list = final_target_list_choice[2];
                if (nowPosition[1] == 3) final_target_list = final_target_list_choice[3];
            } 
        }
        if (nowPosition[1] > 25) final_target_list = final_target_list_choice[4];
    }
	finalTarget = final_target_list[0];
    srand(time(NULL));
}

void refreshColorMap()
{
    // std::cout << "Refresh Color: " << std::endl;
	for (int i = 0; i < LENGTH; i++)
	{
		for (int j = 0; j < LENGTH; j++)
		{
			if (gameInfo->GetCellColor(i, j) == THUAI4::ColorType::Invisible)
				continue;
			if (gameInfo->GetCellColor(i, j) == THUAI4::ColorType::None)
			{
				colorMap[i][j] = 0;
			}
			else if (gameInfo->GetCellColor(i, j) == gameInfo->GetSelfTeamColor())
                colorMap[i][j] = 1;
			else
                colorMap[i][j] = -1;				
		}
	}
}

void refreshPlayers()
{
	for (auto p = players.begin(); p != players.end();)
	{
		auto self = gameInfo->GetSelfInfo();
		if (getPointToPointDistance(p->second.x, p->second.y, self->x, self->y)  < 5000)
			players.erase(p++);
        else
            p++;
	}
	auto new_players = gameInfo->GetCharacters();
	auto self = gameInfo->GetSelfInfo();
    for (auto p : new_players)
	{
		if (p->teamID == self->teamID)
			continue;
		if (players.find(p->guid) != players.end())
			players.erase(p->guid);
		players.insert(std::make_pair(p->guid, THUAI4::Character(*p)));
	}
}


void refreshProps()
{
	for (auto p = props.begin(); p != props.end(); )
	{
		auto self = gameInfo->GetSelfInfo();
		if (getPointToPointDistance(p->second.x, p->second.y, self->x, self->y)  < 5000)
			props.erase(p++);
        else
            p++;
	}
	auto new_props = gameInfo->GetProps();
	for (auto p : new_props)
	{
		if (props.find(p->guid) != props.end())
			props.erase(p->guid);
		props.insert(std::make_pair(p->guid, THUAI4::Prop(*p)));
	}
}

void updateInfo(GameApi& g)
{
    if (frame == 0) 
        initialization(g);
    frame++;
    isAct = false;
    canStepUnColored = true;
    unColoredDistance = 10;
    gameInfo = &g;
    auto self = g.GetSelfInfo();
    nowPosition = {CordToGrid(self->x), CordToGrid(self->y)};
    nowBulletNum = self->bulletNum;
    refreshColorMap();
	refreshPlayers();
	refreshProps();
    if (colorMap[nowPosition[0]][nowPosition[1]] != 1) tiredFrame++;
    else tiredFrame = 0;
	memcpy(dynamicMap, defaultMap, sizeof(defaultMap));
	for (auto p : gameInfo->GetCharacters()){
		if (p->guid == g.GetSelfInfo()->guid)
			continue;
		if (p->teamID != g.GetSelfInfo()->teamID)
			continue;
		auto tmp_x = (int)round((double)p->x / 1000.0);
		auto tmp_y = (int)round((double)p->y / 1000.0);
		dynamicMap[tmp_x][tmp_y] = dynamicMap[tmp_x - 1][tmp_y] = dynamicMap[tmp_x - 1][tmp_y - 1] = dynamicMap[tmp_x][tmp_y - 1] = 1;
	}
    dijkstra({int(self->x), int(self->y)});
}

void updateEnd()
{
    auto self = gameInfo->GetSelfInfo();
    lastX = self->x;
    lastY = self->y;
    lastPosition = nowPosition;
}

void pickAction()
{
    if (isAct == true) return;
    auto self = gameInfo->GetSelfInfo();
    if (self->propType != THUAI4::PropType::Null)
    {
        gameInfo->Use();
        lastAction = USE;
    }
    
    auto props = gameInfo->GetProps();
    for (auto prop: props)
    {
        if (CordToGrid(prop->x) == nowPosition[0] && CordToGrid(prop->y) == nowPosition[1])
        {
            gameInfo->Pick(prop->propType);
            // propMap[nowPosition[0]][nowPosition[1]] = 0;
            lastAction = PICK;
        }
    }
}

double getBestExtendAngle()
{
    auto self = gameInfo->GetSelfInfo();
    int bestValue = 0;
    int bestAngle = 0;
    for (auto angle = rand() % 10; angle < 360; angle += 10)
    {
        if (std::abs(lastAttackAngle - angle) < 30) continue;
        bool flag = false;
        for (auto bullet : gameInfo->GetBullets())
        {
            if (bullet->teamID != self->teamID) continue;
            if (std::abs(angleToRadian(angle) - bullet->facingDirection) < 0.6)
                flag = true;
        }
        if (flag) continue;
        int value = 0;
        auto lastX = nowPosition[0];
        auto lastY = nowPosition[1];
        for (auto distance = 1000; distance < 100000; distance += 100)
        {
            auto angleR = angleToRadian(angle);
            auto targetX = CordToGrid(self->x + cos(angleR) * distance);
            auto targetY = CordToGrid(self->y + sin(angleR) * distance);
            if (targetX < 0 || targetX > 49 || targetY < 0 || targetY > 49) break;
            if (lastX == targetX && lastY == targetY)
                continue;
            lastX = targetX;
            lastY = targetY;
            int block = defaultMap[targetX][targetY];
            // for (int i = std::max(0, targetX - 1); i <= std::min(49, targetX + 1); ++i)
            //     for (int j = std::max(0, targetY - 1); j <= std::min(49, targetY + 1); ++j)
            //         block += defaultMap[i][j];
            if (block > 0) break;
            if (colorMap[targetX][targetY] == -1) value += 2;
            else if (colorMap[targetX][targetY] == 0) value += 1;
        }
        if (value >= bestValue)
        {
            bestValue = value;
            bestAngle = angle;
        }
    }
    
    auto lastX = nowPosition[0];
    auto lastY = nowPosition[1];
    // std::cout << "Updated extend value: " << std::endl;
    for (auto distance = 1000; distance < 100000; distance ++)
    {
        auto angleR = angleToRadian(bestAngle);
        auto targetX = CordToGrid(self->x + cos(angleR) * distance);
        auto targetY = CordToGrid(self->y + sin(angleR) * distance);
        if (targetX < 0 || targetX > 49 || targetY < 0 || targetY > 49) break;
        if (lastX == targetX && lastY == targetY) continue;
        lastX = targetX;
        lastY = targetY;
        int block = defaultMap[targetX][targetY];
        // for (int i = std::max(0, targetX - 1); i <= std::min(49, targetX + 1); ++i)
        //     for (int j = std::max(0, targetY - 1); j <= std::min(49, targetY + 1); ++j)
        //         block += defaultMap[i][j];
        if (block > 0) break;
        colorMap[targetX][targetY] = 1;
    }
    // std::cout << std::endl;
    // std::cout << "Best Value: " << bestValue << " Best Angle: " << bestAngle << std::endl; 
    lastAttackAngle = bestAngle;
    return angleToRadian(bestAngle);
}

double attackEnemyAngle()
{
    double res = -1, minDistance = MAX_DISTANCE + 1;
	auto self = gameInfo->GetSelfInfo();
    for (auto player: players)
    {
        std::cout << "Player: " << player.second.guid << " " << player.second.x << " " << player.second.y;
        if (self->teamID == player.second.teamID || player.second.hp <= 0 || player.second.isDying) continue;
        double distance = getPointToPointDistance(self->x, self->y, player.second.x, player.second.y);
        if (distance < minDistance)
        {
            minDistance = distance;
            attackTime = int(minDistance / 18. + 0.5);
            attackHp = player.second.hp;
            res = getPointToPointAngle(self->x, self->y, player.second.x, player.second.y);
        }
	}
    // std::cout << "Attack Angle: " << res << std::endl;
    if (job == PURPLE_FISH) return -1;
    if (job == EGG_MAN && minDistance > 1000 * sqrt(2)) return -1;
    return res;
}

void avoidBullet()
{
    if (isAct) return;
    auto self = gameInfo->GetSelfInfo();
    double unitMove = self->moveSpeed / 50.0;
    double rad = self->facingDirection;
    for (auto bullet : gameInfo->GetBullets())
    {
        if (bullet->teamID == self->teamID) continue;
        double bulletUnitMove = bullet->moveSpeed / 50.0;
        double bulletRad = bullet->facingDirection;
        double dx = self->x + unitMove * cos(rad) - bullet->x - bulletUnitMove * cos(bulletRad);
        double dy = self->y + unitMove * sin(rad) - bullet->y - bulletUnitMove * sin(bulletRad);
		double delx = self->x - bullet->x;
		double dely = self->y - bullet->y;
		double distance = sqrt(delx *delx + dely * dely);
		double bomb = bulletBomb[(int)bullet->bulletType];
		double cosRad = std::abs(delx * cos(bulletRad) + dely * sin(bulletRad)) / sqrt(delx *delx + dely * dely);
		double tanRad = tan(acos(cosRad));
		if(tanRad * distance > bomb)
		{
			int type = (int)bullet->bulletType;
			// if((dx * dx + dy * dy) < ((bulletBomb[type] * 1000 + bullet->moveSpeed + self-> radius) * (bulletBomb[type] * 1000 + bullet->moveSpeed + self-> radius) * (bullet->moveSpeed / self->moveSpeed)))
			{
				double avoidRad = 0;
				if((dx * cos(M_PI / 2 + bulletRad) + dy * sin(M_PI / 2 + bulletRad)) < 0)
					avoidRad = - M_PI / 2 + bulletRad;
				else
					avoidRad = M_PI / 2 + bulletRad;
				if(bulletNow.size() <= 3)
					bulletNow.push({bullet->teamID, bullet->facingDirection, bullet->moveSpeed, avoidRad, self->x, self->y, type});
			}
		}
		// if(atan(std::abs(dx)/std::abs(dy)) + bulletRad < M_PI)
		// {
		//     unsigned char block1 = defaultMap[int((dx + cos(M_PI/2 + bulletRad) * 1000)/1000)][int((dy + sin(M_PI/2 + bulletRad) * 1000)/1000)];
		//     unsigned char block2 = defaultMap[int((dx + cos(- M_PI/2 + bulletRad) * 1000)/1000)][int((dy + sin(- M_PI/2 + bulletRad) * 1000)/1000)];
		// }
    }
	if(!bulletNow.empty())
	{
		bulletCache bullet = bulletNow.top();
		gameInfo->MovePlayer(50, bullet.avoidRad);
		lastAction = MOVE;
		isAct = true;
		double dx = self->x - bullet.tmpx;
		double dy = self->y - bullet.tmpy;
		double bomb = bulletBomb[bullet.type] * 1000;
		printf("%d %d\n", dx * dx + dy * dy, bomb * bomb);
		bool safe = true;
		for (auto bullet : gameInfo->GetBullets())
		{
			if (bullet->teamID == self->teamID) continue;
			double bulletUnitMove = bullet->moveSpeed / 50.0;
			double bulletRad = bullet->facingDirection;
			double dx = self->x - bullet->x;
			double dy = self->y - bullet->y;
			if((std::abs(dx * cos(bulletRad) + dy * sin(bulletRad)) / sqrt(dx *dx + dy * dy)) > 0.5)
			{
				safe = false;
			}
		}
		if(dx *dx + dy * dy > 2 * bomb * bomb || safe)
		{
			printf("%d %d", dx * dx + dy * dy, bomb * bomb);
			while (!bulletNow.empty()) bulletNow.pop();
		}
	}

}

void attackAction()
{
    if (isAct) return;
    auto self = gameInfo->GetSelfInfo();
    // std::cout << "Bullet Num: " << nowBulletNum << std::endl;
    if (nowBulletNum == 0) return;

    double angle = attackEnemyAngle();
    if (angle < 0)
    {
        if (job == PURPLE_FISH && nowBulletNum > 1) angle = getBestExtendAngle();
        else return;
    }
    // std::cout << "Attack Angle(Radius): " << angle << std::endl;
    if (job == PURPLE_FISH)
    {
        gameInfo->Attack(0, angle);
        nowBulletNum--;
    }
    else
    {
        while (nowBulletNum > 0 && attackHp > 0)
        {
            attackHp -= 3500;
            nowBulletNum--;
            gameInfo->Attack(attackTime, angle);
        }
    }
    lastAction = ATTACK;
}

Position findBestTarget()
{
    double minDistance = MAX_DISTANCE + 1;
    double directDistance = 0;
    double targetAngleR = 0;
    Position bestTarget = {24, 24};
    auto self = gameInfo->GetSelfInfo();
	static int count = 0;

    // First to approach or aloof the enemy
    for (auto player : players)
    {   
        if (self->teamID == player.second.teamID || player.second.hp <= 0 ||player.second.isDying) continue;
        double distance =  distance_table[CordToGrid(player.second.x)][CordToGrid(player.second.y)];
        if (distance < minDistance)
        {
            minDistance = distance;
            directDistance = getPointToPointDistance(self->x, self->y, player.second.x, player.second.y);
            targetAngleR = getPointToPointAngle(self->x, self->y, player.second.x, player.second.y);
            if (job == PURPLE_FISH)
            {
                targetAngleR -= M_PI;
                if (targetAngleR < 0) targetAngleR += 2 * M_PI;
                int tempTargetX = CordToGrid(self->x + cos(targetAngleR) * 5000);
                int tempTargetY = CordToGrid(self->y + sin(targetAngleR) * 5000);
                bestTarget = {std::max(0, std::min(49, tempTargetX)), std::max(0, std::min(49, tempTargetY))};
                unColoredDistance = 1;
            }
            else if (job == EGG_MAN || job == MONKEY_DOCTOR)
            {
                bestTarget = {CordToGrid(player.second.x), CordToGrid(player.second.y)};
                if (nowBulletNum != self->maxBulletNum) 
                {
                    unColoredDistance = 10;
                    canStepUnColored = false;
                }
                else unColoredDistance = 1;
            }
        }
    }

    std::cout << "Min Distance 1: " << minDistance << std::endl;
    std::cout << "Best Target 1: " << bestTarget[0] << " " << bestTarget[1] << std::endl;

    if (directDistance > 4998 && job == PURPLE_FISH)
        minDistance = MAX_DISTANCE + 1;
    if (nowBulletNum == 0 && job == MONKEY_DOCTOR && colorMap[nowPosition[0]][nowPosition[1]] != 1)
        minDistance = MAX_DISTANCE + 1;

    if (minDistance < MAX_DISTANCE + 1)
    {
        if (job == PURPLE_FISH)
        {
            auto countBegin = count;
            double minAngleOffset = M_PI;
            int bestCount = count;
            for (int i = 0; i < final_target_list.size(); ++i)
            {
                auto tempTargetAngleR = getPointToPointAngle(self->x, self->y, GridToCord(final_target_list[count][0]), GridToCord(final_target_list[count][1]));
                auto angleOffset = fabs(tempTargetAngleR - targetAngleR); 
                if (angleOffset > M_PI) angleOffset = 2 * M_PI - angleOffset;
                std::cout << count << " " << angleOffset << std::endl;
                if (angleOffset < minAngleOffset)
                {
                    minAngleOffset = angleOffset;
                    bestCount = count;
                }
                count = (count + 1) % final_target_list.size();
            }
            count = bestCount;
            finalTarget = final_target_list[count];   
        }
        return bestTarget;
    }

    // Second to get items
    for (auto prop : props)
	{
		double distance = distance_table[CordToGrid(prop.second.x)][CordToGrid(prop.second.y)];
		if (distance < minDistance)
		{
			// std::cout << "Prop at: " << i << " " << j << std::endl;
			minDistance = distance;
            targetAngleR = getPointToPointAngle(self->x, self->y, prop.second.x, prop.second.y);
			bestTarget = {CordToGrid(prop.second.x), CordToGrid(prop.second.y)};
            if (job == PURPLE_FISH && nowBulletNum < 1) canStepUnColored = false;
		}
	}
    
    std::cout << "Min Distance 2: " << minDistance << std::endl;
    std::cout << "Best Target 2: " << bestTarget[0] << " " << bestTarget[1] << std::endl;

    if (minDistance < MAX_DISTANCE + 1)
        return bestTarget;

    // No item or enemy, set a defualt target 
    if (getGridDistance(nowPosition, finalTarget) <= 5)
	{
		count = (count + 1) % final_target_list.size();
        finalTarget = final_target_list[count];   
	}
    if (job == PURPLE_FISH)
    {
        canStepUnColored = false;
        unColoredDistance = 10;
    }
    else if (job == EGG_MAN || job == MONKEY_DOCTOR)
    {
        if (nowBulletNum != self->maxBulletNum) 
        {
            unColoredDistance = 10;
            canStepUnColored = false;
        }
        else unColoredDistance = 1;
    }
    bestTarget = finalTarget;

    std::cout << "Min Distance 3: " << minDistance << std::endl;
    std::cout << "Best Target 3: " << bestTarget[0] << " " << bestTarget[1] << std::endl;

    return bestTarget;
}

void moveAction()
{
    if (isAct) return;
    auto self = gameInfo->GetSelfInfo();
    // std::cout << "???" << std::endl;
    nowTarget = findBestTarget();
    if (nowTarget == nowPosition)
    {
        // std::cout << "WAIT(1)!: " << std::endl;
        lastAction = WAIT;
        isAct = true;
        return;
    }
    dijkstra({int(self->x), int(self->y)});
    auto l = searchWayFromMap(nowPosition, nowTarget);
    if (job == PURPLE_FISH && tiredFrame > 20 && nowBulletNum > 0)
        gameInfo->Attack(0, getPointToPointAngle(self->x, self->y, GridToCord(nowPosition[0]), GridToCord(nowPosition[1]))); 
    if (*l.begin() == DONT_MOVE)
    {
        std::cout << "WAIT(2)!: " << std::endl;
        lastAction = WAIT;
        isAct = true;
        return;
    }
    double angle = getMoveAngle(l.begin());
    nextPosition = {nowPosition[0] + dirX[*l.begin()], nowPosition[1] + dirY[*l.begin()]};
    // std::cout << "MoveSpeed: " << self->moveSpeed << std::endl;
    // std::cout << "nextPositionX: " << nextPosition[0] << std::endl;
    // std::cout << "nextPositionY: " << nextPosition[1] << std::endl;
    // std::cout << "MoveAngle: " << angle << std::endl;
    gameInfo->MovePlayer(50, angle);
    lastAction = MOVE;
    isAct = true;
}

int getSelfPlayerID()
{
	int self_playerID = 0;
	for (auto i = 0; i < gameInfo->GetPlayerGUIDs()[gameInfo->GetSelfInfo()->teamID].size(); ++i) 
	{
		if (gameInfo->GetSelfInfo()->guid == gameInfo->GetPlayerGUIDs()[gameInfo->GetSelfInfo()->teamID][i])
		{
			self_playerID = i;
			break;
		}
	}
	return self_playerID;
}

void sendMessage()
{
	auto self_playerID = getSelfPlayerID();
	auto self = gameInfo->GetSelfInfo();
	for (int i = 0; i < 3; ++i)
	{
		if (i == self_playerID)
			continue;
		std::string strToSend;
		strToSend.push_back((char)CordToGrid(self->x));
		strToSend.push_back((char)CordToGrid(self->y));
		auto new_players = gameInfo->GetCharacters();
		for (auto p : new_players)
		{
			if (p->teamID == self->teamID)
				continue;
			if (p->isDying)
				continue;
			strToSend.push_back((char)CordToGrid(p->x));
			strToSend.push_back((char)CordToGrid(p->y));
			strToSend.push_back((char)(p->hp / 100));
		}
		gameInfo->Send(i, strToSend);
	}
}

void recieveMessage()
{
	for (auto it = players.begin(); it != players.end();)
	{
		if (it->first < 0)
			players.erase(it++);
		else
			it++;
	}
	auto self_playerID = getSelfPlayerID();
	std::string recieveStr;
	std::cout << "Recieved : " << std::endl;
	while (gameInfo->TryGetMessage(recieveStr))
	{
		int i = 2;
		for(; i < recieveStr.size();)
		{
			THUAI4::Character c;
			c.teamID = !gameInfo->GetSelfInfo()->teamID;
			c.x = GridToCord((int)recieveStr[i]);
			++i;
			c.y = GridToCord((int)recieveStr[i]);
			++i;
			c.hp = int(recieveStr[i]) * 100;
			++i;
			players.insert(std::make_pair(-i, c));
		}
	}
}

void debugInfo()
{
	std::cout << "Now Frame " << frame << " Elapse: " << processEnd - processBegin << std::endl;
	std::cout << "================================" << std::endl;
	auto self = gameInfo->GetSelfInfo();
    std::cout << "tiredFrame: " << tiredFrame << std::endl;
    std::cout << "NowBulletNum: " << nowBulletNum << std::endl;
    std::cout << "maxBulletNum: " << self->maxBulletNum << std::endl;
    std::cout << "CanStepUnColored: " << canStepUnColored << std::endl;
    std::cout << "unColoredDistance: " << unColoredDistance << std::endl;
    std::cout << "Last Action: " << lastAction << std::endl;
    std::cout << "Last Attack Angle: " << lastAttackAngle << std::endl;
    std::cout << "LastPosition(Cord): (" << lastX << "," << lastY << ")" << std::endl;
	std::cout << "NowPosition(Cord): (" << self->x << "," << self->y << ")" << std::endl;
    std::cout << "NowPosition(Grid): (" << nowPosition[0] << "," << nowPosition[1] << ")" << std::endl;
    std::cout << "NextPosition(Grid): (" << nextPosition[0] << "," << nextPosition[1] << ")" << std::endl;
    std::cout << "Now target is: (" << nowTarget[0] << ", " << nowTarget[1] << ")" << std::endl;
    std::cout << "Final target is: (" << finalTarget[0] << ", " << finalTarget[1] << ")" << std::endl;
    std::cout << "Neighbour Color Map: " << std::endl;
    std::cout << int(colorMap[std::max(nowPosition[0] - 1, 0)][std::max(nowPosition[1] - 1, 0)]) << " ";
    std::cout << int(colorMap[std::max(nowPosition[0] - 1, 0)][nowPosition[1]]) << " ";
    std::cout << int(colorMap[std::max(nowPosition[0] - 1, 0)][std::min(nowPosition[1] + 1, 49)]) << " " << std::endl;
    std::cout << int(colorMap[nowPosition[0]][std::max(nowPosition[1] - 1, 0)]) << " ";
    std::cout << int(colorMap[nowPosition[0]][nowPosition[1]]) << " ";
    std::cout << int(colorMap[nowPosition[0]][std::min(nowPosition[1] + 1, 49)]) << " " << std::endl;
    std::cout << int(colorMap[std::min(nowPosition[0] + 1, 49)][std::max(nowPosition[1] - 1, 0)]) << " ";
    std::cout << int(colorMap[std::min(nowPosition[0] + 1, 49)][nowPosition[1]]) << " ";
    std::cout << int(colorMap[std::min(nowPosition[0] + 1, 49)][std::min(nowPosition[1] + 1, 49)]) << " " << std::endl;
    std::cout << "Neighbour Distance: " << std::endl;
    std::cout << distance_table[std::max(nowPosition[0] - 1, 0)][std::max(nowPosition[1] - 1, 0)] << " ";
    std::cout << distance_table[std::max(nowPosition[0] - 1, 0)][nowPosition[1]] << " ";
    std::cout << distance_table[std::max(nowPosition[0] - 1, 0)][std::min(nowPosition[1] + 1, 49)] << " " << std::endl;
    std::cout << distance_table[nowPosition[0]][std::max(nowPosition[1] - 1, 0)] << " ";
    std::cout << distance_table[nowPosition[0]][nowPosition[1]] << " ";
    std::cout << distance_table[nowPosition[0]][std::min(nowPosition[1] + 1, 49)] << " " << std::endl;
    std::cout << distance_table[std::min(nowPosition[0] + 1, 49)][std::min(nowPosition[1] - 1, 49)] << " ";
    std::cout << distance_table[std::min(nowPosition[0] + 1, 49)][nowPosition[1]] << " ";
    std::cout << distance_table[std::min(nowPosition[0] + 1, 49)][std::min(nowPosition[1] + 1, 49)] << " " << std::endl;
	std::cout << "Players:" << std::endl;
	for (auto p : players)
	{
		std::cout << "\t" << p.first << " : " << p.second.x << " , " << p.second.y << "hp: " << p.second.hp << std::endl;
	}
	std::cout << "================================" << std::endl;
    std::cout << std::endl;
}

void AI::play(GameApi& g)
{
    processBegin = clock();
    updateInfo(g);
    sendMessage();
	recieveMessage();
    pickAction();
    avoidBullet();
    attackAction();
    moveAction();
    updateEnd();
    processEnd = clock();
    debugInfo();
    usleep(50000);
}