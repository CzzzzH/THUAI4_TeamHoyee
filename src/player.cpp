#include "AI.h"
#include "Constants.h"

//为假则play()调用期间游戏状态更新阻塞，为真则只保证当前游戏状态不会被状态更新函数与GameApi的方法同时访问
extern const bool asynchronous = true;

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
enum Action {MOVE, ATTACK, PICK, WAIT};
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
static char colorValueMap[LENGTH][LENGTH];
static int frame = 0;
static int lastX, lastY;
static bool isAct;

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

/****************************************/
/*                                      */
/*        Overload Functions            */
/*                                      */
/****************************************/

std::array<int, 2> operator+(const std::array<int, 2>& p1, const std::array<int, 2>& p2)
{
	return std::array<int, 2>{p1[0] + p2[0], p1[1] + p2[1]};
}
std::array<int, 2> operator-(const std::array<int, 2>& p1, const std::array<int, 2>& p2)
{
	return std::array<int, 2>{p1[0] - p2[0], p1[1] - p2[1]};
}
std::array<int, 2> operator-(const std::array<int, 2>& p1)
{
	return std::array<int, 2>{-p1[0], -p1[1]};
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
    return sqrt((source[0] - target[0]) * (source[0] - target[0]) + (source[1] - target[1]) *(source[1] - target[1]));
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
    return int(value / 1000);
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

void dijkstra(const std::array<int, 2>& point)
{
	memset(route, 0, sizeof(route));
	memset(colorValueMap, 0, sizeof(colorValueMap));
	// std::cout << sizeof(route) << std::endl;
	__gnu_pbds::priority_queue<frontier_node, std::greater<frontier_node>> frontier;
	memset(distance_table, -1, sizeof(distance_table));
	memset(dont_search, 0, sizeof(dont_search));

	frontier.push(frontier_node{ 0, point });
	distance_table[point[0]][point[1]] = 0;
	colorValueMap[point[0]][point[1]] = colorMap[point[0]][point[1]];
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
		bool avaiable[8];
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
				frontier.push(frontier_node{ distance + neighbor_distance, p });
			else
				frontier.modify(iter, frontier_node{ distance + neighbor_distance, p });
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
	std::list<unsigned char> result;
	while (1)
	{
		func(end);
		if (end == start)
			break;
		result.push_front(route[end[0]][end[1]]);
		end = end - operate[route[end[0]][end[1]]];
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
        finalTarget = {50 - CordToGrid(self->x), 50 - CordToGrid(self->y)};
    }
    
    srand(time(NULL));
}

void refreshColorMap()
{
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
	auto new_players = gameInfo->GetCharacters();
	for (auto p : new_players)
	{
		if (players.find(p->guid) != players.end())
			players.erase(p->guid);
		players.insert(std::make_pair(p->guid, THUAI4::Character(*p)));
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
    // std::cout << "Now Position: " << nowPosition[0] << " " << nowPosition[1] << std::endl;
    dijkstra(nowPosition);
}

void updateEnd()
{
    auto self = gameInfo->GetSelfInfo();
    lastX = self->x;
    lastY = self->y;
    lastPosition = nowPosition;
}

int getBestExtendAngle()
{
    auto currentX = gameInfo->GetSelfInfo()->x;
    auto currentY = gameInfo->GetSelfInfo()->y;
    int bestValue = 0;
    int bestAngle = 0;
    for (auto angle = 0; angle < 360; angle++)
    {
        int value = 0;
        for (auto distance = 1000; distance < 500000; distance += 1000)
        {
            auto angleR = angleToRadian(angle);
            auto targetX = currentX + cos(angleR) * distance;
            auto targetY = currentY + sin(angleR) * distance;
            int block = defaultMap[CordToGrid(targetX)][CordToGrid(targetY)];
            if (block == 1) break;
            int color = colorMap[CordToGrid(targetX)][CordToGrid(targetY)];
            if (color == -1) value += 2;
            else if (color == 0) value += 1;
        }
        if (value > bestValue)
        {
            bestValue = value;
            bestAngle = angle;
        }
    }
    
    for (auto distance = 1000; distance < 500000; distance += 1000)
    {
        auto angleR = angleToRadian(bestAngle);
        auto targetX = currentX + cos(angleR) * distance;
        auto targetY = currentY + sin(angleR) * distance;
        int block = defaultMap[CordToGrid(targetX)][CordToGrid(targetY)];
        if (block == 1) break;
        colorMap[CordToGrid(targetX)][CordToGrid(targetY)] = 1;
    }

    return bestAngle;
}

void attackAction()
{
    if (isAct) return;
    auto self = gameInfo->GetSelfInfo();
    if (self->bulletNum == 0)
        currentState = WAIT_BULLET;
    else
    {
        if (job == PURPLE_FISH)
        {
            auto bestExtendAngle = getBestExtendAngle();
            gameInfo->Attack(0, angleToRadian(bestExtendAngle));
            lastAction = ATTACK;
            isAct = true;
            currentState = EXPAND_FIELD;
        }
        else if (job == MONKEY_DOCTOR)
        {
            currentState = SEARCH_ENEMY;
        }
    }
}

void correctPosition()
{
    if (isAct) return;
    auto self = gameInfo->GetSelfInfo();
    if (!(lastAction == MOVE && lastX == self->x && lastY == self->y)) return;
    auto centerX = GridToCord(nowPosition[0]); 
    auto centerY = GridToCord(nowPosition[1]); 
    auto angle = getPointToPointAngle(self->x, self->y, centerX, centerY);
    auto distance = getPointToPointDistance(self->x, self->y, centerX, centerY);
    uint32_t time = uint32_t(distance / double(self->moveSpeed) * 1000);
    std::cout << "Correct Position Time: " << time << std::endl;
    gameInfo->MovePlayer(time, angle);
    lastAction = MOVE;
    isAct = true;
}

Position findFarthestTarget()
{
    Position bestPosition = nowPosition;
    double maxDistance = 0;
    auto self = gameInfo->GetSelfInfo();
    int bonus = 0;
    for (auto i = 0; i < 50; ++i)
        for (auto j = 0; j < 50; ++j)
        {
            if (defaultMap[i][j] == 1) continue;
            if (nowPosition[0] == i && nowPosition[1] == j) continue;
            Position newPosition = {i, j};
            bonus = - int(getGridDistance(nowTarget, {i, j}) * 0.5);
            if (distance_table[i][j] + bonus < maxDistance) continue;
            auto l = searchWayFromMap(nowPosition, {i, j});
            int k = 0;
            auto nextX = self->x;
            auto nextY = self->y;
            bool flag = true;
            for (auto it = l.begin(); it != l.end(); ++it)
            {
                if (k == 2) break;
                nextX += self->moveSpeed / 20. * cos(getMoveAngle(it));
                nextY += self->moveSpeed / 20. * sin(getMoveAngle(it));
                if (defaultMap[i][j] == 1 || colorMap[CordToGrid(nextX)][CordToGrid(nextY)] != 1)
                {
                    flag = false;
                    break;
                }
                k++;
            }
            if (flag)
            {
                maxDistance = distance_table[i][j] + bonus;
                bestPosition = {i, j};
            }
        }
    std::cout << "Bonus: " << bonus << std::endl;
    return bestPosition;
}

Position findNearestTeamColor()
{   
    double minDistance = 200;
    Position res;
    for (auto i = 0; i < 50; ++i)
        for (auto j = 0; j < 50; ++j)
        {
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
        if (colorMap[nowPosition[0]][nowPosition[1]] == 0) nowTarget = findNearestTeamColor();
        else nowTarget = findFarthestTarget();
        if (nowTarget == nowPosition)
        {
            lastAction = WAIT;
            return;
        }
        auto l = searchWayFromMap(nowPosition, nowTarget);
        double angle = getMoveAngle(l.begin());
        double nextX = self->x + 1000. * cos(angle);
        double nextY = self->y + 1000. * sin(angle);
        nextPosition = {CordToGrid(nextX), CordToGrid(nextY)};
        // std::cout << "Angle: " << angle << std::endl;
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
	std::cout << "Now Frame " << frame << std::endl;
	std::cout << "================================" << std::endl;
	auto self = gameInfo->GetSelfInfo();
    std::cout << "LastPosition(Cord): (" << lastX << "," << lastY << ")." << std::endl;
	std::cout << "NowPosition(Cord): (" << self->x << "," << self->y << ")." << std::endl;
    std::cout << "NowPosition(Grid): (" << nowPosition[0] << "," << nextPosition[1] << ")" << std::endl;
    std::cout << "NextPosition(Grid): (" << nextPosition[0] << "," << nextPosition[1] << ")" << std::endl;
    std::cout << "Now target is: (" << nowTarget[0] << ", " << nowTarget[1] << ")" << std::endl;
    // std::cout << "Final target is: (" << finalTarget[0] << ", " << finalTarget[1] << ")" << std::endl;
	std::cout << "================================" << std::endl;
    std::cout << std::endl;
}

void AI::play(GameApi& g)
{
    updateInfo(g);
    attackAction();
    correctPosition();
    moveAction();
    updateEnd();
    debugInfo();
    usleep(100000);
}
