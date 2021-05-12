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

enum Action {MOVE, ATTACK, PICK, USE, WAIT};
enum Job {PURPLE_FISH, EGG_MAN};

/****************************************/
/*                                      */
/*          Global Variables             */
/*                                      */
/****************************************/

extern const THUAI4::JobType playerJob = THUAI4::JobType::Job3; // Purple Fish
// extern const THUAI4::JobType playerJob = THUAI4::JobType::Job5; // Egg Man

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

const int INF_DISTANCE = 0x3f3f3f3f;
const unsigned char DONT_MOVE = 8;
static unsigned char route[LENGTH][LENGTH];
static double distance_table[LENGTH][LENGTH];
static char colorMap[LENGTH][LENGTH];
static char colorValueMap[LENGTH][LENGTH];
static double areaValue[4];
static int frame = 0;
static int lastX, lastY, lastAttackAngle;
static bool isAct, getItem;
static clock_t processBegin, processEnd;

Position lastPosition, nowPosition, nextPosition, nowTarget, finalTarget;
static std::vector<std::vector<Position>> final_target_list_choice;
std::vector<Position> final_target_list;


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
		for (int x = std::max(p[0] - i, 0) ; x <= std::min(p[0] + i, 49) ; ++x)
		{
			for (int y = std::max(p[1] - i, 0); y <= std::min(p[1] + i, 49) ; y += 2*i)
				if(!dynamicMap[x][y])
				{
					return {x, y};
				}
		}
		for (int y = std::max(p[1] - i + 1, 0) ; y <= std::min(p[1] + i - 1, 49) ; ++y)
		{
			for (int x = std::max(p[0] - i, 0) ; x <= std::min(p[0] + i, 49) ; x += 2*i)
				if(!dynamicMap[x][y])
				{
					return {x, y};
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
    if (playerJob == THUAI4::JobType::Job5) job = EGG_MAN;

    if (job == EGG_MAN)
    {
        final_target_list_choice= {
            {{8, 8}, {8, 41}, {41, 8}, {41, 41}},
            {{8, 8}, {8, 41}, {41, 8}, {41, 41}},
            {{8, 8}, {8, 41}, {41, 8}, {41, 41}},
            {{8, 8}, {8, 41}, {41, 8}, {41, 41}}
            };
    }
    else if (job == PURPLE_FISH)
    {
        final_target_list_choice= {
            {{11, 5}, {40, 6}, {36, 27}, {11, 27}},
            {{7, 18}, {42, 16}, {42, 41}, {6, 40}},
            {{21, 15}, {34, 11}, {27, 25}},
            {{25, 33}, {25, 45}}
            };
    }

    nowPosition = { CordToGrid(self->x), CordToGrid(self->y) };
    if (nowPosition[0] == 2) final_target_list = final_target_list_choice[0];
    else if (nowPosition[0] == 3) final_target_list = final_target_list_choice[1];
    else if (nowPosition[0] == 46) final_target_list = final_target_list_choice[2];
    else if (nowPosition[0] == 47) final_target_list = final_target_list_choice[3];
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
	auto new_players = gameInfo->GetCharacters();
	auto self = gameInfo->GetSelfInfo();
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
    canStepUnColored = true;
    unColoredDistance = 10;
    gameInfo = &g;
    auto self = g.GetSelfInfo();
    nowPosition = {CordToGrid(self->x), CordToGrid(self->y)};
    refreshColorMap();
	refreshPlayers();
	refreshProps();
	memcpy(dynamicMap, defaultMap, sizeof(defaultMap));
	for (auto p : players){
		if (p.second.guid == g.GetSelfInfo()->guid)
			continue;
		auto tmp_x = (int)round((double)p.second.x / 1000.0);
		auto tmp_y = (int)round((double)p.second.y / 1000.0);
		dynamicMap[tmp_x][tmp_y] = dynamicMap[tmp_x - 1][tmp_y] = dynamicMap[tmp_x - 1][tmp_y - 1] = dynamicMap[tmp_x][tmp_y - 1] = 1;
	}
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
                // propMap[nowPosition[0]][nowPosition[1]] = 0;
                lastAction = PICK;
                isAct = true;
            }
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
    double res = -1, minDistance = INF_DISTANCE;
    auto targetPlayers = gameInfo->GetCharacters();
	auto self = gameInfo->GetSelfInfo();
    for (auto player: targetPlayers)
    {
        if (self->teamID == player->teamID) continue;
        double distance = getPointToPointDistance(self->x, self->y, player->x, player->y);
        if (distance < minDistance)
        {
            minDistance = distance;
            res = getPointToPointAngle(self->x, self->y, player->x, player->y);
        }
	}
    // std::cout << "Attack Angle: " << res << std::endl;
    if (job == EGG_MAN && minDistance > 1000 * sqrt(2)) return -1;
    return res;
}

void avoidBullet()
{
    if (isAct) return;
    auto self = gameInfo->GetSelfInfo();
    double unitMove = self->moveSpeed / 50.0;
    double rad = self->facingDirection;
    // std::cout << "Bullet Size: " << gameInfo->GetBullets().size() << std::endl;
    for (auto bullet : gameInfo->GetBullets())
    {
        if (bullet->teamID == self->teamID) continue;
        double bulletUnitMove = bullet->moveSpeed / 50.0;
        double bulletRad = bullet->facingDirection;
        double dx = self->x + unitMove * cos(rad) - bullet->x - bulletUnitMove * cos(bulletRad);
        double dy = self->y + unitMove * sin(rad) - bullet->y - bulletUnitMove * sin(bulletRad);
        if((dx * dx + dy * dy) < ((bullet->radius + self-> radius) * (bullet->radius + self-> radius) * 20))
        {
            if((std::abs(dx * cos(bulletRad) + dy * sin(bulletRad)) / sqrt(dx *dx + dy * dy)) > 0.7)
            {
                if((dx * cos(M_PI / 2 + bulletRad) + dy * sin(M_PI / 2 + bulletRad)) < 0)
                {
                    gameInfo->MovePlayer(50, - M_PI / 2 + bulletRad);
                }
                else
                {
                    gameInfo->MovePlayer(50, M_PI / 2 + bulletRad);
                }
            }

            // if(atan(std::abs(dx)/std::abs(dy)) + bulletRad < M_PI)
            // {
            //     unsigned char block1 = defaultMap[int((dx + cos(M_PI/2 + bulletRad) * 1000)/1000)][int((dy + sin(M_PI/2 + bulletRad) * 1000)/1000)];
            //     unsigned char block2 = defaultMap[int((dx + cos(- M_PI/2 + bulletRad) * 1000)/1000)][int((dy + sin(- M_PI/2 + bulletRad) * 1000)/1000)];
            // }
            lastAction = MOVE;
            isAct = true;
            break;
        }
    }
}

void attackAction()
{
    if (isAct) return;
    auto self = gameInfo->GetSelfInfo();
    if (self->bulletNum == 0) return;

    double angle = attackEnemyAngle();
    if (angle < 0)
    {
        if (job == PURPLE_FISH) angle = getBestExtendAngle();
        else if (job == EGG_MAN) return;
    }
    // std::cout << "Attack Angle(Radius): " << angle << std::endl;
    gameInfo->Attack(0, angle);
    lastAction = ATTACK;
    isAct = true;
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

Position findBestTarget()
{
    int minDistance = INF_DISTANCE;
    Position bestTarget;
    auto self = gameInfo->GetSelfInfo();
	static int count = 0;

    // First to get items
    for (auto prop : props)
	{
		double distance = getPointToPointDistance(self->x, self->y, prop.second.x, prop.second.y);
		if (distance < minDistance)
		{
			// std::cout << "Prop at: " << i << " " << j << std::endl;
			minDistance = distance;
            unColoredDistance = 1;
			bestTarget = {(int)prop.second.x, (int)prop.second.y};
		}
	}

    // Second to approach or aloof the enemy
    for (auto player : players)
    {
        if (self->teamID == player.second.teamID) continue;
        double distance =  getPointToPointDistance(self->x, self->y, player.second.x, player.second.y);
        if (distance < minDistance)
        {
            minDistance = distance;
            double targetAngleR = getPointToPointAngle(self->x, self->y, player.second.x, player.second.y);
            if (job == PURPLE_FISH)
            {
                targetAngleR -= M_PI;
                if (targetAngleR < 0) targetAngleR += 2 * M_PI;
                int tempTargetX = (int)(self->x + cos(targetAngleR) * 2000);
                int tempTargetY = (int)(self->y + sin(targetAngleR) * 2000);
                bestTarget = {std::max(0, std::min(49, tempTargetX)), std::max(0, std::min(49, tempTargetY))};
                unColoredDistance = 1;
            }
            else if (job == EGG_MAN)
            {
                bestTarget = {CordToGrid(player.second.x), CordToGrid(player.second.y)};
                if (self->bulletNum  == 0) 
                {
                    unColoredDistance = 10;
                    canStepUnColored = false;
                }
                else if (self->bulletNum == 1) unColoredDistance = 3;
                else unColoredDistance = 1;
            }
        }
    }

    // std::cout << "Min Distance: " << minDistance << std::endl;
    // std::cout << "Best Target: " << bestTarget[0] << " " << bestTarget[1] << std::endl;
    
    if (minDistance < INF_DISTANCE)
        return bestTarget;

    // No item or enemy, set a defualt target 
    if (getGridDistance(nowPosition, finalTarget) <= 2)
	{
		count = (count + 1) % final_target_list.size();
        finalTarget = final_target_list[count];
        if (job == PURPLE_FISH) canStepUnColored = false;
        else if (job == EGG_MAN)
        {
            if (self->bulletNum  < 2) 
            {
                unColoredDistance = 10;
                canStepUnColored = false;
            }
            else if (self->bulletNum == 2) unColoredDistance = 5;
            else if (self->bulletNum == 3) unColoredDistance = 2;
            else unColoredDistance = 1;
        }   
	}
    bestTarget = finalTarget;
    return bestTarget;
}

void moveAction()
{
    if (isAct) return;
    auto self = gameInfo->GetSelfInfo();
    
    nowTarget = findBestTarget();
    if (nowTarget == nowPosition)
    {
        // std::cout << "WAIT(1)!: " << std::endl;
        lastAction = WAIT;
        return;
    }
    dijkstra({int(self->x), int(self->y)});
    auto l = searchWayFromMap(nowPosition, nowTarget);
    if (*l.begin() == DONT_MOVE)
    {
        // std::cout << "WAIT(2)!: " << std::endl;
        lastAction = WAIT;
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

void debugInfo()
{
	std::cout << "Now Frame " << frame << " Elapse: " << processEnd - processBegin << std::endl;
	std::cout << "================================" << std::endl;
	auto self = gameInfo->GetSelfInfo();
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
	std::cout << "================================" << std::endl;
    std::cout << std::endl;
}

void AI::play(GameApi& g)
{
    processBegin = clock();
    updateInfo(g);
    pickAction();
    avoidBullet();
    attackAction();
    correctPosition();
    moveAction();
    updateEnd();
    processEnd = clock();
    // debugInfo();
    usleep(50000);
}
