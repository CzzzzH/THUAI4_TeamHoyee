#include <stdio.h>
#include <limits>
#include <list>
// #include "MainWindow.h"
#include "input_poly.h"
#include <iostream>

using namespace std;

#include "polypartition.h"
// #include "kdtree.h"
#include <ext/pb_ds/priority_queue.hpp>
#include <unordered_set>
const int times = 10;

struct frontier_node
{
	double distance;
	TPPLPolyList::iterator iter;
	bool operator<(const frontier_node &n) const
	{
		return distance < n.distance;
	}
	bool operator>(const frontier_node &n) const
	{
		return distance > n.distance;
	}
};

double pointToLineDistance(const TPPLPoint &point, const TPPLPoint &linePoint1, const TPPLPoint &linePoint2)
{
	double A = linePoint2.y - linePoint1.y;
	double B = linePoint1.x - linePoint2.x;
	double C = linePoint2.x * linePoint1.y - linePoint1.x * linePoint2.y;
	return fabs(A * point.x + B * point.y + C) / sqrt(A * A + B * B);
}
double length(const TPPLPoint &p1) { return sqrt(p1.x * p1.x + p1.y * p1.y); }
double cosAngle(const TPPLPoint &p1, const TPPLPoint &p2) { return (p1.x * p2.x + p1.y * p2.y) / (length(p1) * length(p2)); }

int isOnLinesRight(const TPPLPoint &p, const TPPLPoint &linePoint1, const TPPLPoint &linePoint2)
{
	/* positive means point on line's right side
		   negitive means point on line's left side
		   zero means point on line
		*/
	return (linePoint2.y - linePoint1.y) * p.x + (linePoint1.x - linePoint2.x) * p.y + (linePoint2.x * linePoint1.y - linePoint1.x * linePoint2.y);
}

bool isPointInPoly(const TPPLPoly &poly, const TPPLPoint &point)
{
	bool result = true;
	for (auto p = poly.begin(); p != poly.end(); ++p)
	{
		auto tmp_p = p;
		tmp_p = tmp_p == --poly.end() ? poly.begin() : ++tmp_p;
		if (isOnLinesRight(point, *p, *tmp_p) > 0)
		{
			result = false;
			// std::cout << "(" << point.x << "," << point.y << ") is on right" << std::endl;
			break;
		}
	}
	return result;
}

TPPLPoint computeCenter(TPPLPoly &poly)
{
	double x_sum = 0;
	double y_sum = 0;
	for (auto p : poly)
	{
		// std::cout << "round  : (" << p.x << "," << p.y << ") " << std::endl;
		x_sum += p.x;
		y_sum += p.y;
	}
	return TPPLPoint{x_sum / poly.GetNumPoints(), y_sum / poly.GetNumPoints()};
}

TPPLPoint twoLineIntesection(const TPPLPoint &line1p1, const TPPLPoint &line1p2,
							 const TPPLPoint &line2p1, const TPPLPoint &line2p2)
{
	double A1 = line1p1.y - line1p2.y;
	double B1 = line1p2.x - line1p1.x;
	double C1 = line1p1.x * line1p2.y - line1p2.x * line1p1.y;
	double A2 = line2p1.y - line2p2.y;
	double B2 = line2p2.x - line2p1.x;
	double C2 = line2p1.x * line2p2.y - line2p2.x * line2p1.y;

	double D = A1 * B2 - A2 * B1;
	return TPPLPoint{(B1 * C2 - B2 * C1) / D, (A2 * C1 - A1 * C2) / D};
}
std::list<TPPLPoint> searchWay(TPPLPolyList &polyList,
							   const std::unordered_multimap<TPPLPoint, TPPLPolyList::iterator, hash_point> &point_poly_map,
							   const TPPLPolyList::iterator &begin,
							   const TPPLPolyList::iterator &end)
{
	TPPLPoint centerpoint = computeCenter(*begin);
	std::cout << "center  : (" << centerpoint.x << "," << centerpoint.y << ") " << std::endl;
	TPPLPoint endpoint = computeCenter(*end);
	std::cout << "end  : " << &*end << "(" << endpoint.x << ", " << endpoint.y << ") " << std::endl;

	__gnu_pbds::priority_queue<frontier_node, std::greater<frontier_node>>
		frontier;
	std::unordered_set<TPPLPoly *> non_searchable;
	std::unordered_map<TPPLPoly *, double> distance_table;
	std::unordered_map<TPPLPoly *, std::tuple<std::list<TPPLPoint>::iterator, std::list<TPPLPoint>::iterator, TPPLPolyList::iterator>> route_table;

	frontier.push(frontier_node{0, begin});
	distance_table.insert(std::make_pair(&*begin, 0));
	while (true)
	{
		std::cout << "heap size : " << frontier.size() << std::endl;
		auto poly_iter = frontier.top().iter;
		frontier.pop();
		std::cout << "pop frontier :  " << &*poly_iter << std::endl;
		double distance = distance_table.at(&*poly_iter);
		non_searchable.insert(&*poly_iter);
		if (poly_iter == end)
			break;
		for (auto vertex = poly_iter->begin(); vertex != poly_iter->end(); ++vertex)
		{
			// std::cout << "    vertex  : (" << vertex->x << "," << vertex->y << ") " << std::endl;
			TPPLPolyList::iterator poly2 = polyList.end();
			auto next_vertex = vertex;
			next_vertex = next_vertex == --poly_iter->end() ? poly_iter->begin() : ++next_vertex;
			// std::cout << "next vertex  : (" << next_vertex->x << "," << next_vertex->y << ") " << std::endl;
			{
				auto range1 = point_poly_map.equal_range(*vertex);
				for (auto r1 = range1.first; r1 != range1.second; ++r1)
				{
					auto range2 = point_poly_map.equal_range(*next_vertex);
					for (auto r2 = range2.first; r2 != range2.second; ++r2)
					{
						if (&*(r1->second) == &*(r2->second) && &*(r1->second) != &*poly_iter)
						{
							poly2 = r1->second;
							break;
						}
					}
					if (poly2 != polyList.end())
						break;
				}
				if (poly2 == polyList.end())
					continue;
			}
			// std::cout << "    poly2 :  " << &*poly2 << std::endl;
			if (non_searchable.find(&*poly2) != non_searchable.end())
			{
				// std::cout << "    non_searchable  " << std::endl;
				continue;
			}
			double neighbor_distance;
			if (poly_iter == begin)
			{
				neighbor_distance = TPPLPartition::Distance(centerpoint, (*vertex + *next_vertex) / 2);
			}
			else
			{
				auto tmp_iter = std::get<0>(route_table.at(&*poly_iter));
				auto tmp2_iter = std::get<1>(route_table.at(&*poly_iter));
				neighbor_distance = TPPLPartition::Distance((*tmp_iter + *tmp2_iter) / 2, (*vertex + *next_vertex) / 2);
			}
			// std::cout << "    neighbor distance :  " << neighbor_distance << std::endl;
			if (distance_table.find(&*poly2) != distance_table.end() &&
				distance + neighbor_distance > distance_table.at(&*poly2))
			{
				// std::cout << "    do not update distance  " << std::endl;
				continue;
			}
			auto it_f = frontier.begin();
			for (; it_f != frontier.end(); ++it_f)
			{
				if (it_f->iter == poly2)
				{
					// std::cout << "    modify frontier  " << std::endl;
					frontier.modify(it_f, frontier_node{distance + neighbor_distance, poly2});
					break;
				}
			}
			if (it_f == frontier.end())
			{
				// std::cout << "    push into frontier  " << std::endl;
				frontier.push(frontier_node{distance + neighbor_distance, poly2});
			}
			if (distance_table.find(&*poly2) != distance_table.end())
			{
				// std::cout << "    modify distance  " << std::endl;
				distance_table.at(&*poly2) = distance + neighbor_distance;
			}
			else
			{
				// std::cout << "    insert distance  " << std::endl;
				distance_table.insert(std::make_pair(&*poly2, distance + neighbor_distance));
			}
			if (route_table.find(&*poly2) != route_table.end())
				route_table.erase(&*poly2);
			for (std::list<TPPLPoint>::iterator p = poly2->points.begin(); p != poly2->points.end(); ++p)
				if (p->x == next_vertex->x && p->y == next_vertex->y)
				{
					auto tmp = p;
					tmp = tmp == --(poly2->points.end()) ? poly2->points.begin() : ++tmp;
					route_table.insert(
						std::make_pair(
							&*poly2,
							std::tuple<
								std::list<TPPLPoint>::iterator,
								std::list<TPPLPoint>::iterator,
								TPPLPolyList::iterator>(p, tmp, poly_iter)));
					std::cout << "    route : (" << p->x << "," << p->y << ") "
							  << "(" << tmp->x << "," << tmp->y << ") " << std::endl;
				}

			// std::cout << "  complete one vertex  " << std::endl;
		}
	}

	std::list<TPPLPoint> result;
	result.push_back(endpoint);
	auto back_node = end;
	while (true)
	{
		if (back_node == begin)
			break;
		auto back = route_table.at(&*back_node);
		std::cout << "(" << std::get<0>(back)->x << "," << std::get<0>(back)->y << ") "
				  << "(" << std::get<1>(back)->x << "," << std::get<1>(back)->y << ") " << std::endl;
		result.push_back(TPPLPoint((*std::get<0>(back) + *std::get<1>(back)) / 2));
		back_node = std::get<2>(back);
	}
	// result.push_back(centerpoint);
	return result;
}

std::list<TPPLPoint> searchWay(TPPLPolyList &polyList,
							   const std::unordered_multimap<TPPLPoint, TPPLPolyList::iterator, hash_point> &point_poly_map,
							   const TPPLPolyList::iterator &begin,
							   const TPPLPoint &end)
{
	TPPLPoint centerpoint = computeCenter(*begin);
	std::cout << "center  : (" << centerpoint.x << "," << centerpoint.y << ") " << std::endl;
	TPPLPoint endpoint = end;
	std::cout << "end  : "
			  << "(" << endpoint.x << ", " << endpoint.y << ") " << std::endl;

	__gnu_pbds::priority_queue<frontier_node, std::greater<frontier_node>>
		frontier;
	std::unordered_set<TPPLPoly *> non_searchable;
	std::unordered_map<TPPLPoly *, double> distance_table;
	std::unordered_map<TPPLPoly *, std::tuple<TPPLPoint, TPPLPolyList::iterator>> route_table;

	frontier.push(frontier_node{0, begin});
	distance_table.insert(std::make_pair(&*begin, 0));
	auto poly_iter = polyList.end();
	while (true)
	{
		std::cout << "heap size : " << frontier.size() << std::endl;
		poly_iter = frontier.top().iter;
		frontier.pop();
		std::cout << "pop frontier :  " << &*poly_iter << std::endl;
		double distance = distance_table.at(&*poly_iter);
		non_searchable.insert(&*poly_iter);
		if (isPointInPoly(*poly_iter, end))
		{
			std::cout << "get you !!!" << std::endl;
			break;
		}
		for (auto vertex = poly_iter->begin(); vertex != poly_iter->end(); ++vertex)
		{
			std::cout << "    vertex  : (" << vertex->x << "," << vertex->y << ") " << std::endl;
			TPPLPolyList::iterator poly2 = polyList.end();
			auto next_vertex = vertex;
			next_vertex = next_vertex == --poly_iter->end() ? poly_iter->begin() : ++next_vertex;
			// std::cout << "next vertex  : (" << next_vertex->x << "," << next_vertex->y << ") " << std::endl;
			{
				auto range1 = point_poly_map.equal_range(*vertex);
				for (auto r1 = range1.first; r1 != range1.second; ++r1)
				{
					auto range2 = point_poly_map.equal_range(*next_vertex);
					for (auto r2 = range2.first; r2 != range2.second; ++r2)
					{
						if (&*(r1->second) == &*(r2->second) && &*(r1->second) != &*poly_iter)
						{
							poly2 = r1->second;
							break;
						}
					}
					if (poly2 != polyList.end())
						break;
				}
				if (poly2 == polyList.end())
					continue;
			}
			std::cout << "    poly2 :  " << &*poly2 << std::endl;
			if (non_searchable.find(&*poly2) != non_searchable.end())
			{
				std::cout << "    non_searchable  " << std::endl;
				continue;
			}

			TPPLPoint nowPoint;
			if (poly_iter == begin)
			{
				nowPoint = centerpoint;
			}
			else
			{
				nowPoint = std::get<0>(route_table.at(&*poly_iter));
			}
			TPPLPoint contactPoint;
			std::cout << "now : (" << nowPoint.x << "," << nowPoint.y << ") "
					  << " end : (" << endpoint.x << "," << endpoint.y << ") "
					  << "(" << vertex->x << "," << vertex->y << ") "
					  << "(" << next_vertex->x << "," << next_vertex->y << ") " << std::endl;
			if (TPPLPartition::Intersects(nowPoint, endpoint, *vertex, *next_vertex))
			{
				contactPoint = twoLineIntesection(nowPoint, endpoint, *vertex, *next_vertex);
			}
			else
			{
				if (cosAngle(*vertex - nowPoint, endpoint - nowPoint) > cosAngle(*next_vertex - nowPoint, endpoint - nowPoint))
					contactPoint = *vertex;
				else
					contactPoint = *next_vertex;
			}
			std::cout << "contact : (" << contactPoint.x << "," << contactPoint.y << ") ";
			double neighbor_distance = TPPLPartition::Distance(nowPoint, contactPoint);
			// if (poly_iter == begin)
			// {
			// 	neighbor_distance = TPPLPartition::Distance(centerpoint, (*vertex + *next_vertex) / 2);
			// }
			// else
			// {
			// 	auto tmp_iter = std::get<0>(route_table.at(&*poly_iter));
			// 	auto tmp2_iter = std::get<1>(route_table.at(&*poly_iter));
			// 	neighbor_distance = TPPLPartition::Distance((*tmp_iter + *tmp2_iter) / 2, (*vertex + *next_vertex) / 2);
			// }
			std::cout << "    neighbor distance :  " << neighbor_distance << std::endl;
			if (distance_table.find(&*poly2) != distance_table.end() &&
				distance + neighbor_distance > distance_table.at(&*poly2))
			{
				std::cout << "    do not update distance  " << std::endl;
				continue;
			}
			auto it_f = frontier.begin();
			for (; it_f != frontier.end(); ++it_f)
			{
				if (it_f->iter == poly2)
				{
					std::cout << "    modify frontier  " << std::endl;
					frontier.modify(it_f, frontier_node{distance + neighbor_distance + TPPLPartition::Distance(contactPoint, endpoint), poly2});
					break;
				}
			}
			if (it_f == frontier.end())
			{
				std::cout << "    push into frontier  " << std::endl;
				frontier.push(frontier_node{distance + neighbor_distance + TPPLPartition::Distance(contactPoint, endpoint), poly2});
			}
			if (distance_table.find(&*poly2) != distance_table.end())
			{
				std::cout << "    modify distance  " << std::endl;
				distance_table.at(&*poly2) = distance + neighbor_distance;
			}
			else
			{
				std::cout << "    insert distance  " << std::endl;
				distance_table.insert(std::make_pair(&*poly2, distance + neighbor_distance));
			}
			if (route_table.find(&*poly2) != route_table.end())
				route_table.erase(&*poly2);
			for (std::list<TPPLPoint>::iterator p = poly2->points.begin(); p != poly2->points.end(); ++p)
				if (p->x == next_vertex->x && p->y == next_vertex->y)
				{
					auto tmp = p;
					tmp = tmp == --(poly2->points.end()) ? poly2->points.begin() : ++tmp;
					route_table.insert(
						std::make_pair(
							&*poly2,
							std::tuple<
								TPPLPoint,
								TPPLPolyList::iterator>(contactPoint, poly_iter)));
					std::cout << "    route : (" << p->x << "," << p->y << ") "
							  << "(" << tmp->x << "," << tmp->y << ")   " << &*poly_iter << std::endl;
				}

			std::cout << "  complete one vertex  " << std::endl;
		}
	}

	std::list<TPPLPoint> result;
	result.push_back(endpoint);
	auto back_node = poly_iter;
	int i = 0;
	while (true)
	{
		if (back_node == begin)
			break;
		auto back = route_table.at(&*back_node);
		std::cout << "back node : " << &*back_node << " (" << std::get<0>(back).x << "," << std::get<0>(back).y << ") "
				  << "  back : " << &*std::get<1>(back) << std::endl;
		result.push_back(TPPLPoint(std::get<0>(back)));
		back_node = std::get<1>(back);
		i++;
		if (i > 100)
			break;
	}
	result.push_back(centerpoint);
	return result;
}

std::list<TPPLPoint> searchWay(TPPLPolyList &polyList,
							   const std::unordered_multimap<TPPLPoint, TPPLPolyList::iterator, hash_point> &point_poly_map,
							   const TPPLPoint &begin,
							   const TPPLPoint &end, double radius)
{
	TPPLPoint centerpoint = begin;
	std::cout << "center  : (" << centerpoint.x << "," << centerpoint.y << ") " << std::endl;
	TPPLPoint endpoint = end;
	std::cout << "end  : "
			  << "(" << endpoint.x << ", " << endpoint.y << ") " << std::endl;

	__gnu_pbds::priority_queue<frontier_node, std::greater<frontier_node>>
		frontier;
	std::unordered_set<TPPLPoly *> non_searchable;
	std::unordered_map<TPPLPoly *, double> distance_table;
	std::unordered_map<TPPLPoly *, std::tuple<TPPLPoint, TPPLPolyList::iterator>> route_table;

	auto begin_poly_iter = polyList.begin();
	for (; begin_poly_iter != polyList.end(); ++begin_poly_iter)
	{
		if (isPointInPoly(*begin_poly_iter, begin))
			break;
	}

	frontier.push(frontier_node{0, begin_poly_iter});
	distance_table.insert(std::make_pair(&*begin_poly_iter, 0));
	auto poly_iter = polyList.end();
	while (true)
	{
		// std::cout << "heap size : " << frontier.size() << std::endl;
		poly_iter = frontier.top().iter;
		frontier.pop();
		// std::cout << "pop frontier :  " << &*poly_iter << std::endl;
		double distance = distance_table.at(&*poly_iter);
		non_searchable.insert(&*poly_iter);
		if (isPointInPoly(*poly_iter, end))
		{
			// std::cout << "get you !!!" << std::endl;
			break;
		}
		for (auto vertex = poly_iter->begin(); vertex != poly_iter->end(); ++vertex)
		{
			// std::cout << "    vertex  : (" << vertex->x << "," << vertex->y << ") " << std::endl;
			TPPLPolyList::iterator poly2 = polyList.end();
			auto next_vertex = vertex;
			next_vertex = next_vertex == --poly_iter->end() ? poly_iter->begin() : ++next_vertex;
			// std::cout << "next vertex  : (" << next_vertex->x << "," << next_vertex->y << ") " << std::endl;
			{
				auto range1 = point_poly_map.equal_range(*vertex);
				for (auto r1 = range1.first; r1 != range1.second; ++r1)
				{
					auto range2 = point_poly_map.equal_range(*next_vertex);
					for (auto r2 = range2.first; r2 != range2.second; ++r2)
					{
						if (&*(r1->second) == &*(r2->second) && &*(r1->second) != &*poly_iter)
						{
							poly2 = r1->second;
							break;
						}
					}
					if (poly2 != polyList.end())
						break;
				}
				if (poly2 == polyList.end())
					continue;
			}
			// std::cout << "    poly2 :  " << &*poly2 << std::endl;
			if (non_searchable.find(&*poly2) != non_searchable.end())
			{
				// std::cout << "    non_searchable  " << std::endl;
				continue;
			}

			if (TPPLPartition::Distance(*vertex, *next_vertex) < 2 * radius)
			{
				// std::cout << " too narrow to pass" << std::endl;
				continue;
			}

			TPPLPoint nowPoint;
			if (poly_iter == begin_poly_iter)
			{
				nowPoint = centerpoint;
			}
			else
			{
				nowPoint = std::get<0>(route_table.at(&*poly_iter));
			}
			TPPLPoint contactPoint;
			// std::cout << "now : (" << nowPoint.x << "," << nowPoint.y << ") "
			// 		  << " end : (" << endpoint.x << "," << endpoint.y << ") "
			// 		  << "(" << vertex->x << "," << vertex->y << ") "
			// 		  << "(" << next_vertex->x << "," << next_vertex->y << ") " << std::endl;
			TPPLPoint vertex_close = *vertex + (*next_vertex - *vertex) / TPPLPartition::Distance(*next_vertex, *vertex) * radius;
			TPPLPoint next_vertex_close = *next_vertex + (*vertex - *next_vertex) / TPPLPartition::Distance(*next_vertex, *vertex) * radius;
			if (TPPLPartition::Intersects(nowPoint, endpoint, vertex_close, next_vertex_close))
			{
				contactPoint = twoLineIntesection(nowPoint, endpoint, vertex_close, next_vertex_close);
			}
			else
			{
				if (cosAngle(vertex_close - nowPoint, endpoint - nowPoint) > cosAngle(next_vertex_close - nowPoint, endpoint - nowPoint))
					contactPoint = vertex_close;
				else
					contactPoint = next_vertex_close;
			}
			// std::cout << "contact : (" << contactPoint.x << "," << contactPoint.y << ") ";
			double neighbor_distance = TPPLPartition::Distance(nowPoint, contactPoint);
			// std::cout << "    neighbor distance :  " << neighbor_distance << std::endl;
			if (distance_table.find(&*poly2) != distance_table.end() &&
				distance + neighbor_distance > distance_table.at(&*poly2))
			{
				// std::cout << "    do not update distance  " << std::endl;
				continue;
			}
			auto it_f = frontier.begin();
			for (; it_f != frontier.end(); ++it_f)
			{
				if (it_f->iter == poly2)
				{
					// std::cout << "    modify frontier  " << std::endl;
					frontier.modify(it_f, frontier_node{distance + neighbor_distance + TPPLPartition::Distance(contactPoint, endpoint), poly2});
					break;
				}
			}
			if (it_f == frontier.end())
			{
				// std::cout << "    pushs into frontier  " << std::endl;
				frontier.push(frontier_node{distance + neighbor_distance + TPPLPartition::Distance(contactPoint, endpoint), poly2});
			}
			if (distance_table.find(&*poly2) != distance_table.end())
			{
				// std::cout << "    modify distance  " << std::endl;
				distance_table.at(&*poly2) = distance + neighbor_distance;
			}
			else
			{
				// std::cout << "    insert distance  " << std::endl;
				distance_table.insert(std::make_pair(&*poly2, distance + neighbor_distance));
			}
			if (route_table.find(&*poly2) != route_table.end())
				route_table.erase(&*poly2);
			for (std::list<TPPLPoint>::iterator p = poly2->points.begin(); p != poly2->points.end(); ++p)
				if (p->x == next_vertex->x && p->y == next_vertex->y)
				{
					auto tmp = p;
					tmp = tmp == --(poly2->points.end()) ? poly2->points.begin() : ++tmp;
					route_table.insert(
						std::make_pair(
							&*poly2,
							std::tuple<
								TPPLPoint,
								TPPLPolyList::iterator>(contactPoint, poly_iter)));
					// std::cout << "    route : (" << p->x << "," << p->y << ") "
					// 		  << "(" << tmp->x << "," << tmp->y << ")   " << &*poly_iter << std::endl;
				}

			// std::cout << "  complete one vertex  " << std::endl;
		}
	}

	std::list<TPPLPoint> result;
	result.push_back(endpoint);
	auto back_node = poly_iter;
	int i = 0;
	while (true)
	{
		if (back_node == begin_poly_iter)
			break;
		auto back = route_table.at(&*back_node);
		// std::cout << "back node : " << &*back_node << " (" << std::get<0>(back).x << "," << std::get<0>(back).y << ") "
		// 		  << "  back : " << &*std::get<1>(back) << std::endl;
		result.push_back(TPPLPoint(std::get<0>(back)));
		back_node = std::get<1>(back);
		i++;
		if (i > 100)
			break;
	}
	result.push_back(centerpoint);
	return result;
}

// int main(int argc, char **argv)
// {
// 	QApplication app(argc, argv);
// 	MainWindow mainWindow;
// 	mainWindow.resize(500, 500);

// 	TPPLPartition pp;

// 	list<TPPLPoly> testpolys, result;

// 	auto p_list = point_list7;

// 	// ReadPolyList("test_input1.txt", &testpolys);

// 	for (auto l1 = p_list.begin(); l1 != p_list.end(); ++l1)
// 	{
// 		TPPLPoly poly;
// 		poly.Init();
// 		if (l1 != p_list.begin())
// 			poly.SetHole(true);
// 		// int i = 0;
// 		for (auto p : *l1)
// 		{
// 			poly.push_back({p[0], p[1]});
// 		}
// 		testpolys.push_back(poly);
// 	}

// 	for (auto poly : testpolys)
// 	{
// 		for (auto k = poly.begin(); k != poly.end(); ++k)
// 		{
// 			mainWindow.points.push_back(std::make_pair(
// 				(std::array<double, 2>){times * k->x, times * k->y},
// 				Qt::blue));
// 		}
// 	}

// 	printf("Testing ConvexPartition_HM: ");
// 	std::unordered_multimap<TPPLPoint, TPPLPolyList::iterator, hash_point> point_poly_map;
// 	pp.ConvexPartition_HM(&testpolys, &result, point_poly_map);

// 	for (auto poly_iter = result.begin(); poly_iter != result.end(); poly_iter++)
// 	{
// 		std::cout << &*poly_iter << "  :  ";
// 		auto k = poly_iter->begin();
// 		for (; k != --poly_iter->end(); ++k)
// 		{
// 			std::cout << "(" << k->x << "," << k->y << ")   ";
// 			auto tmp = k;
// 			++tmp;
// 			mainWindow.lines.push_back(std::make_pair(
// 				(std::array<double, 4>){times * k->x, times * k->y, times * tmp->x, times * tmp->y},
// 				Qt::green));
// 		}
// 		std::cout << "(" << k->x << "," << k->y << ")   " << std::endl;
// 		mainWindow.lines.push_back(std::make_pair(
// 			(std::array<double, 4>){times * k->x, times * k->y, times * poly_iter->begin()->x, times * poly_iter->begin()->y},
// 			Qt::green));
// 	}

// 	mainWindow.show();

// 	// auto l = searchWay(result, point_poly_map, result.begin(), ------result.end());
// 	// auto l = searchWay(result, point_poly_map, result.begin(), TPPLPoint{2, 2});
// 	auto l = searchWay(result, point_poly_map, TPPLPoint{12, 40}, TPPLPoint{2, 2}, 1);

// 	auto k = l.begin();
// 	for (; k != --l.end(); ++k)
// 	{
// 		auto tmp = k;
// 		++tmp;
// 		mainWindow.lines.push_back(std::make_pair(
// 			(std::array<double, 4>){times * k->x, times * k->y, times * tmp->x, times * tmp->y},
// 			Qt::red));
// 	}

// 	std::cout << isPointInPoly(*result.begin(), TPPLPoint{40, 30}) << std::endl;
// 	std::cout << TPPLPartition::Intersects(TPPLPoint{10, 10}, TPPLPoint{50, 50},
// 										   TPPLPoint{40, 10}, TPPLPoint{30, 25})
// 			  << std::endl;
// 	std::cout << pointToLineDistance(TPPLPoint{10, 10}, TPPLPoint{50, 50},
// 									 TPPLPoint{40, 10})
// 			  << std::endl;

// 	return app.exec();
// }
