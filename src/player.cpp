#include "AI.h"
#include "Constants.h"

//为假则play()调用期间游戏状态更新阻塞，为真则只保证当前游戏状态不会被状态更新函数与GameApi的方法同时访问
extern const bool asynchronous = false;

#include <random>
#include <iostream>
#include <unordered_map>
// #include "polypartition.h"
// #include "PolygonConvexPartition.h"
// #include "map_poly.h"
#include "math.h"
#include <iostream>
#include <unistd.h>
#include <ext/pb_ds/priority_queue.hpp>
#include <unordered_set>
// #include "MainWindow.h"
#include <math.h>
#include <stdlib.h>
#include <ctime>
#include <memory>
#include <list>
#include <string.h>
#include <random>

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

enum State {WAIT_BULLET, EXPAND_FIELD, SEARCH_ENEMY};
enum Action {MOVE, ATTACK, PICK, USE, WAIT};
enum Job {LAZY_GOAT, MONKEY_DOCTOR, PURPLE_FISH};

/****************************************/
/*                                      */
/*          Global Variables             */
/*                                      */
/****************************************/

extern const THUAI4::JobType playerJob = THUAI4::JobType::Job3; //选手职业，选手 !!必须!! 定义此变量来选择职业

const int ZOOM = 10;
const int LENGTH = 50;
static unsigned char defaultMap[LENGTH][LENGTH] = {
	//0                             10                            20                            30                            40                         49
	{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, //0
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0, 1},
	{1, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0, 1},
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
	{1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},

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
	{1, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 0, 1},
	{1, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 0, 0, 1},
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
	{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1} //49
};

std::unordered_map<int64_t, THUAI4::Character> players;
std::unordered_map<int64_t, THUAI4::Prop> props;

static GameApi* gameInfo;
static State currentState;
static Action lastAction;
static Job job;
static unsigned char route[LENGTH][LENGTH];
static double distance_table[LENGTH][LENGTH];
static bool dont_search[LENGTH][LENGTH];
static char colorMap[LENGTH][LENGTH];
static char hasExtend[LENGTH][LENGTH];
static char colorValueMap[LENGTH][LENGTH];
static char propMap[LENGTH][LENGTH];
static char enemyMap[LENGTH][LENGTH];
static double areaValue[4];
static int frame = 0;
static int lastX, lastY, lastAttackAngle;
static bool isAct, getItem;
static clock_t processBegin, processEnd;

Position lastPosition, nowPosition, nextPosition, nowTarget, finalTarget;

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
	memset(route, 8, sizeof(route));
	memset(colorValueMap, 0, sizeof(colorValueMap));
	// std::cout << sizeof(route) << std::endl;
	__gnu_pbds::priority_queue<frontier_node, std::greater<frontier_node>> frontier;
	memset(distance_table, -1, sizeof(distance_table));
	memset(dont_search, 0, sizeof(dont_search));

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
			if (defaultMap[p[i][0]][p[i][1]])
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
			if (defaultMap[p[5][0]][p[5][1]])
				avaiable[4] = 0;
			if (defaultMap[p[7][0]][p[7][1]])
				avaiable[0] = 0;
		}
		else if (decimal_part[1] > 500)
		{
			if (defaultMap[p[3][0]][p[3][1]])
				avaiable[4] = 0;
			if (defaultMap[p[1][0]][p[1][1]])
				avaiable[0] = 0;
		}
		if (decimal_part[0] < 500)
		{
			if (defaultMap[p[3][0]][p[3][1]])
				avaiable[2] = 0;
			if (defaultMap[p[5][0]][p[5][1]])
				avaiable[6] = 0;
		}
		else if (decimal_part[0] > 500)
		{
			if (defaultMap[p[1][0]][p[1][1]])
				avaiable[2] = 0;
			if (defaultMap[p[7][0]][p[7][1]])
				avaiable[6] = 0;
		}
		if (
			(defaultMap[p[1][0]][p[1][1]] && pointToLineDistance(operate[1] * 500 + 500, decimal_part, decimal_part + (operate[3] * 500)) < 600) ||
			(defaultMap[p[5][0]][p[5][1]] && pointToLineDistance(operate[5] * 500 + 500, decimal_part, decimal_part + (operate[3] * 500)) < 600))
			avaiable[3] = avaiable[7] = 0;
		if (
			(defaultMap[p[3][0]][p[3][1]] && pointToLineDistance(operate[3] * 500 + 500, decimal_part, decimal_part + (operate[1] * 500)) < 600) ||
			(defaultMap[p[7][0]][p[7][1]] && pointToLineDistance(operate[7] * 500 + 500, decimal_part, decimal_part + (operate[1] * 500)) < 600))
			avaiable[1] = avaiable[5] = 0;
		for (int i = 0; i < 8; ++i)
		{
			if (!avaiable[i])
				continue;
			auto p = int_part + operate[i];
			double distance = i % 2 ? sqrt(2) : 1;
			frontier.push(frontier_node{distance, p});
            if (colorMap[p[0]][p[1]] != 1) distance_table[p[0]][p[1]] = distance * 10;
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
		for (int i = 0; i < 8; i += 2)
		{
			auto p = searching_point + operate[i];
			if (defaultMap[p[0]][p[1]])
			{
				// std::cout << "setting unavaiable : " << i << std::endl;
				avaiable[i] = 0;
				avaiable[(i + 8 - 1) % 8] = 0;
				avaiable[(i + 1) % 8] = 0;
			}
		}
		for (int i = 1; i < 8; i += 2)
		{
			auto p = searching_point + operate[i];
			if (defaultMap[p[0]][p[1]])
			{
				// std::cout << "setting unavaiable : " << i << std::endl;
				avaiable[i] = 0;
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
				neighbor_distance *= 20;
			if (dont_search[p[0]][p[1]])
				continue;
			if (distance_table[p[0]][p[1]] >= 0 && distance + neighbor_distance > distance_table[p[0]][p[1]])
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

std::list<unsigned char> searchWayFromMap(
	std::array<int, 2> start, std::array<int, 2> end,
	std::function<void(std::array<int, 2>)> func = [](std::array<int, 2> p) {})
{
	// std::cout << "start : " << start[0] << "," << start[1] <<std::endl;
	// std::cout << "end : " << end[0] << "," << end[1] <<std::endl;
	std::list<unsigned char> result;
	while (1)
	{
		func(end);
		if (end == start)
			break;
		result.push_front(route[end[0]][end[1]]);
		// std::cout<<"ddd"<<std::endl;
		end = end - operate[route[end[0]][end[1]]];
		// std::cout<<"eee"<<std::endl;
	}
	return result;
}

void initialization(GameApi& g)
{
    auto self = g.GetSelfInfo();
    
    if (playerJob == THUAI4::JobType::Job2) job = LAZY_GOAT;
    if (playerJob == THUAI4::JobType::Job3) job = PURPLE_FISH;
    if (playerJob == THUAI4::JobType::Job4) job = MONKEY_DOCTOR;

    if (job == PURPLE_FISH)
    {
        finalTarget = {4, 4};
        getItem = false;
        if (CordToGrid(self->x) < 5) finalTarget[0] = 45;
        if (CordToGrid(self->y) < 5) finalTarget[1] = 45;
    }
    
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
            propMap[i][j] = 0;
            enemyMap[i][j] = 0;
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
	auto new_players = gameInfo->GetCharacters();
	auto self = gameInfo->GetSelfInfo();
    for (auto p : new_players)
	{
		if (players.find(p->guid) != players.end())
			players.erase(p->guid);
		players.insert(std::make_pair(p->guid, THUAI4::Character(*p)));
        // std::cout << CordToGrid(p->x) << " " << CordToGrid(p->y) << std::endl;
        if (p->teamID != self->teamID) enemyMap[CordToGrid(p->x)][CordToGrid(p->y)] = 1;
	}
}


void refreshProps()
{
	auto new_props = gameInfo->GetProps();
	for (auto p : new_props)
	{
		if (props.find(p->guid) != props.end())
			props.erase(p->guid);
		props.insert(std::make_pair(p->guid, THUAI4::Prop(*p)));
        // std::cout << CordToGrid(p->x) << " " << CordToGrid(p->y) << std::endl;
        propMap[CordToGrid(p->x)][CordToGrid(p->y)] = 1;
	}
}

void updateInfo(GameApi& g)
{
    if (frame == 0) 
        initialization(g);
    frame++;
    isAct = false;
    gameInfo = &g;
    auto self = g.GetSelfInfo();
    nowPosition = {CordToGrid(self->x), CordToGrid(self->y)};
    refreshColorMap();
	refreshPlayers();
	refreshProps();
    int extendCount = 0;
    for (int i = 0; i < 50; ++i)
        for (int j = 0; j < 50; ++j)
            extendCount += hasExtend[i][j];
    if (extendCount > 1800)
        memset(hasExtend, 0, sizeof(hasExtend));
    dijkstra({int(self->x), int(self->y)});
    // dijkstra({GridToCord(nowPosition[0]), GridToCord(nowPosition[1])});
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
        isAct = true;
    }
    else
    {
        auto props = gameInfo->GetProps();
        for (auto prop: props)
        {
            if (CordToGrid(prop->x) == nowPosition[0] && CordToGrid(prop->y) == nowPosition[1])
            {
                gameInfo->Pick(prop->propType);
                propMap[nowPosition[0]][nowPosition[1]] = 0;
                lastAction = PICK;
                isAct = true;
            }
        }
    }
}

int getBestExtendAngle()
{
    auto self = gameInfo->GetSelfInfo();
    int bestValue = 0;
    int bestAngle = 0;
    for (auto angle = 0; angle < 360; angle ++)
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
        for (auto distance = 0; distance < 100000; distance += 100)
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
            if (hasExtend[targetX][targetY] == 0)
            {
                if (colorMap[targetX][targetY] == -1) value += 2;
                else if (colorMap[targetX][targetY] == 0) value += 1;
            }
            else value -= 1;
        }
        if (value >= bestValue)
        {
            bestValue = value;
            bestAngle = angle;
        }
    }
    
    auto lastX = nowPosition[0];
    auto lastY = nowPosition[1];
    // std::cout << "Updated colorMap origin value: " << std::endl;
    for (auto distance = 0; distance < 100000; distance += 100)
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
        // std::cout << targetX << " " << targetY << " " << int(colorMap[targetX][targetY]) << " | ";
        colorMap[targetX][targetY] = 1;
        hasExtend[targetX][targetY] = 1;
    }
    // std::cout << std::endl;

    // std::cout << "Best Value: " << bestValue << " Best Angle: " << bestAngle << std::endl; 
    lastAttackAngle = bestAngle;
    return angleToRadian(bestAngle);
}

double attackEnemyAngle()
{
    // double res = -1, minDistance = 3000;
    // auto targetPlayers = gameInfo->GetCharacters();
	// auto self = gameInfo->GetSelfInfo();
    // for (auto player: targetPlayers)
    // {
    //     if (self->teamID == player->teamID) continue;
    //     double distance = getPointToPointDistance(self->x, self->y, player->x, player->y);
    //     if (distance < minDistance)
    //     {
    //         minDistance = distance;
    //         res = getPointToPointAngle(self->x, self->y, player->x, player->y);
    //     }
	// }
    // std::cout << "Attack Angle: " << res << std::endl;
    // return res;
    return -1;
}

void attackAction()
{
    if (isAct) return;
    auto self = gameInfo->GetSelfInfo();
    if (self->bulletNum == 0)
        currentState = WAIT_BULLET;
    else
    {
        int angle = attackEnemyAngle();
        if (angle < 0) angle = getBestExtendAngle();
        gameInfo->Attack(0, angle);
        lastAction = ATTACK;
        isAct = true;
        currentState = EXPAND_FIELD;
    }
}

void correctPosition()
{
    if (isAct) return;
    auto self = gameInfo->GetSelfInfo();
    if (!(lastAction == MOVE && lastX == self->x && lastY == self->y)) return;
    auto centerX = GridToCord(nowPosition[0]); 
    auto centerY = GridToCord(nowPosition[1]); 
    auto angleR = getPointToPointAngle(self->x, self->y, centerX, centerY);
    auto distance = getPointToPointDistance(self->x, self->y, centerX, centerY);
    uint32_t time = uint32_t(distance / double(self->moveSpeed) * 1000);
    // std::cout << "Correct Position Angle(Degree): " << radianToAngle(angleR) << std::endl;
    // std::cout << "Correct Position Distance: " << distance << std::endl;
    // std::cout << "Correct Position Time: " << time << std::endl;
    if (frame % 2 == 0) gameInfo->MovePlayer(50, angleR + 0.1);
    else gameInfo->MovePlayer(50, angleR + 0.1);
    lastAction = MOVE;
    isAct = true;
}

void calcAreaValue()
{
    for (auto k = 0; k < 4; ++k)
    {
        areaValue[k] = 1;
        int beginX = (k / 2) * 25;
        int beginY = (k % 2) * 25;
        for (auto i = beginX; i < beginX + 25; ++i)
            for (auto j = beginY; j < beginY + 25; ++j)
            {
                if (defaultMap[i][j]) continue;
                if (colorMap[i][j] == 0) areaValue[k] += 50;
                if (colorMap[i][j] == -1) areaValue[k] += 100;
                if (enemyMap[i][j] == 1) areaValue[k] -= 10000;
            }
    }
}

Position findBestTarget()
{
    int minDistance = 50000;
    Position bestTarget;
    auto self = gameInfo->GetSelfInfo();
    getItem = false;
	std::vector<std::array<int, 2>> final_target_list = {{5, 5}, {43, 12}, {45, 45}, {5, 45}, {8, 17}};
	static int count = 0;

    for (auto i = 0; i < 50; ++i)
        for (auto j = 0; j < 50; ++j)
        {
            if (defaultMap[i][j]) continue;
            if (nowPosition[0] == i && nowPosition[1] == j) continue;
            int distance = distance_table[i][j];
            if (propMap[i][j] == 1 && distance < minDistance)
            {
                // std::cout << "Prop at: " << i << " " << j << std::endl;
                minDistance = distance;
                bestTarget = {i, j};
            }
        }
    // std::cout << "Min Distance: " << minDistance << std::endl;
    // std::cout << "Best Target: " << bestTarget[0] << " " << bestTarget[1] << std::endl;
    if (minDistance < 500 && minDistance > 0)
    {
        getItem = true;
        return bestTarget;
    }
    if (getGridDistance(nowPosition, finalTarget) <= 3)
	{
		count = (count + 1) % final_target_list.size();
        finalTarget = final_target_list[count];
	}
    bestTarget = finalTarget;
    return bestTarget;
}

Position findNearestTeamColor()
{   
    double minDistance = 500000;
    Position res;
    for (auto i = 0; i < 50; ++i)
        for (auto j = 0; j < 50; ++j)
        {
            if (defaultMap[i][j]) continue;
            if (colorMap[i][j] == 1 && distance_table[i][j] < minDistance)
            {
                minDistance = distance_table[i][j];
                res = {i, j};
            }
        }
    return res;
}

void moveAction()
{
    if (isAct) return;
    if (currentState == WAIT_BULLET)
    {
        auto self = gameInfo->GetSelfInfo();
        nowTarget = findBestTarget();
        // std::cout << "Get item: " << getItem << std::endl;
        if (!getItem && colorMap[nowPosition[0]][nowPosition[1]] == 0) 
            nowTarget = findNearestTeamColor();
        if (nowTarget == nowPosition)
        {
            lastAction = WAIT;
            return;
        }
        auto l = searchWayFromMap(nowPosition, nowTarget);
        double angle = getMoveAngle(l.begin());
        nextPosition = {nowPosition[0] + dirX[*l.begin()], nowPosition[1] + dirY[*l.begin()]};
        if (!getItem && colorMap[nextPosition[0]][nextPosition[1]] != 1 && colorMap[nowPosition[0]][nowPosition[1]] != 0)
        {
            lastAction = WAIT;
            return;
        }
        std::cout << "Angle: " << angle << std::endl;
        // std::cout << "MoveSpeed: " << self->moveSpeed << std::endl;
        // std::cout << "nextPositionX: " << nextPosition[0] << std::endl;
        // std::cout << "nextPositionY: " << nextPosition[1] << std::endl;
        gameInfo->MovePlayer(50, angle);
        lastAction = MOVE;
        isAct = true;
    }
}

void debugInfo()
{
	std::cout << "Now Frame " << frame << " Elapse: " << processEnd - processBegin << std::endl;
	std::cout << "================================" << std::endl;
	auto self = gameInfo->GetSelfInfo();
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
    std::cout << distance_table[nowPosition[0]][std::min(nowPosition[1] + 1, 49)] << " ";
    std::cout << distance_table[std::min(nowPosition[0] + 1, 49)][nowPosition[1]] << " ";
    std::cout << distance_table[std::min(nowPosition[0] + 1, 49)][std::min(nowPosition[1] + 1, 49)] << " " << std::endl;
	std::cout << "================================" << std::endl;
    std::cout << std::endl;
}

void AI::play(GameApi& g)
{
    processBegin = clock();
    updateInfo(g);
    pickAction();
    attackAction();
    // correctPosition();
    moveAction();
    updateEnd();
    processEnd = clock();
    debugInfo();
    usleep(50000);
}
