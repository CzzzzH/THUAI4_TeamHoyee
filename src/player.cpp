#include "AI.h"
#include "Constants.h"

//为假则play()调用期间游戏状态更新阻塞，为真则只保证当前游戏状态不会被状态更新函数与GameApi的方法同时访问
extern const bool asynchronous = true;

#include <random>
#include <iostream>
#include <unordered_map>
#include "polypartition.h"
#include "PolygonConvexPartition.h"
#include "map_poly.h"
#include "math.h"
#include <unistd.h>

/* 请于 VS2019 项目属性中开启 C++17 标准：/std:c++17 */

extern const THUAI4::JobType playerJob = THUAI4::JobType::Job0; //选手职业，选手 !!必须!! 定义此变量来选择职业

namespace
{
	[[maybe_unused]] std::uniform_real_distribution<double> direction(0, 2 * 3.1415926);
	[[maybe_unused]] std::default_random_engine e{std::random_device{}()};
}

TPPLPartition pp;
list<TPPLPoly> testpolys, result;
std::unordered_multimap<TPPLPoint, TPPLPolyList::iterator, hash_point> point_poly_map;

bool is_map_done = false;
TPPLPoint target = TPPLPoint{25, 25};

int count_i = 0;

void AI::play(GameApi &g)
{
	auto self = g.GetSelfInfo();

	if (!is_map_done)
	{
		auto p_list = map_poly;
		for (auto l1 = p_list.begin(); l1 != p_list.end(); ++l1)
		{
			TPPLPoly poly;
			poly.Init();
			if (l1 != p_list.begin())
				poly.SetHole(true);
			// int i = 0;
			for (auto p : *l1)
			{
				poly.push_back({p[0], p[1]});
			}
			testpolys.push_back(poly);
		}
		pp.ConvexPartition_HM(&testpolys, &result, point_poly_map);
		is_map_done = true;
		for (auto poly_iter = result.begin(); poly_iter != result.end(); poly_iter++)
		{
			std::cout << &*poly_iter << "  :  ";
			auto k = poly_iter->begin();
			for (; k != --poly_iter->end(); ++k)
			{
				std::cout << "(" << k->x << "," << k->y << ")   ";
				auto tmp = k;
				++tmp;
			}
			std::cout << "(" << k->x << "," << k->y << ")   " << std::endl;
		}
	}

	auto way_list = searchWay(result, point_poly_map, target, TPPLPoint{double(self->x), double(self->y)} / 1000, 2.5);

	if (self->bulletNum)
	{
		g.Attack(100, 0);
	}
	for (auto w : way_list)
	{
		std::cout << "(" << w.x << "," << w.y << ")";
	}
	std::cout << std::endl;
	double angle = atan2((++way_list.begin())->y - (self->y / 1000), (++way_list.begin())->x - (self->x / 1000));
	std::cout << "angle : " << angle << "\n";
	g.MovePlayer(50, angle);
	// g.MovePlayer(50, 1);
	for (int i = 0; i < 50; i++)
	{
		for (int j = 0; j < 50; j++)
		{
			g.GetCellColor(i, j);
		}
	}
	std::cout << "I`m at (" << self->x << "," << self->y << ")." << std::endl;
	// sleep(0.5);
	usleep(100000);
}
