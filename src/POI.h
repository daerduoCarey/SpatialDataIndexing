#include<iostream>
#include<cstdio>
#include<cstdlib>
#include<algorithm>
#include<map>
#include<list>
#include<string>
#include<fstream>
#include<queue>
#include<vector>

#define MaxPOIPerNode 3
#define PI 3.1415926
#define X_SPLIT_TOT 100
#define Y_SPLIT_TOT 100

using namespace std;

struct point
{
	double x,y;

	point() {}
	point(double xx, double yy)
	{
		x = xx;
		y = yy;
	}
	point(const point& obj)
	{
		x = obj.x;
		y = obj.y;
	}
};

struct rectangle
{
	struct point upperLeft,bottomRight;

	rectangle() {}
	rectangle(struct point x, struct point y)
	{
		upperLeft = x;
		bottomRight = y;
	}
	rectangle(const rectangle& obj)
	{
		upperLeft = obj.upperLeft;
		bottomRight = obj.bottomRight;
	}
};

struct circle
{
	struct point center;
	double radius;

	circle(struct point c, double r)
	{
		center = c;
		radius = r;
	}
};

struct POI
{
	string id;
	struct point p;
	int cat;

	POI(string i, int c, double x, double y)
	{
		id = i;
		cat = c;
		p.x = x;
		p.y = y;
	}
};

struct kdNode
{
	bool flag, isLeaf;
	POI* split;
	struct kdNode *left, *right;
	list<POI*> data;
	double minx, miny, maxx, maxy;
	
	kdNode(list<POI*> l)
	{
		data = l;
		isLeaf = true;
	}
	kdNode(bool f, POI* s, kdNode* l, kdNode* r)
	{
		flag = f;
		split = s;
		left = l ;
		right = r;
		isLeaf = false;
	}
};

struct combine
{
	double minx, miny, maxx, maxy;

	combine(double x, double y)
	{
		minx = maxx = x;
		miny = maxy = y;
	}
	combine(double x1, double x2, double y1, double y2)
	{
		minx = x1; maxx = x2;
		miny = y1; maxy = y2;
	}
};

class Search
{
	//All the interfaces available to be invoked
	public:
		Search() {
			x_tot = X_SPLIT_TOT;
			y_tot = Y_SPLIT_TOT;
		}
		~Search() {}

		//using KD-tree method
		list<POI*> KNNQuery(struct point p, int cat, int index);
		list<POI*> RangeQuery(struct circle range, int cat);
		list<POI*> RangeQuery(struct rectangle range, int cat);
		void IndexBuilding(string file_name);

		//using Grid-based method
		list<POI*> KNNQuery2(struct point p, int cat, int index);
		list<POI*> RangeQuery2(struct circle range, int cat);
		list<POI*> RangeQuery2(struct rectangle range, int cat);
		void IndexBuilding2(string file_name);
		void IndexBuilding2(string file_name, int xtot, int ytot);

		//using Brute-scan method
		list<POI*> Scan(struct rectangle, int cat, string file);
		list<POI*> Scan(struct circle, int cat, string file);
		list<POI*> KNNScan(struct point p, int cat, int index, string file);

	private:
		ifstream fin;
		
		//for kd-tree
		map<int, list<POI*>* > table;
		map<int, kdNode* > trees;

		//for grid-based
		map<int, list<POI*>* > table2;
		map<int, vector<list<POI*>*>* > vects;
		double x_max, x_min, y_max, y_min;
		int x_tot, y_tot;
		double x_step, y_step;
		
		rectangle check_validity(const rectangle& range);
		int grid_number(double x, double y);
		bool grid_in_circle(int index, circle range);

		int abs(int x);
		int tot_running_time;

		list<POI*> RangeQuery(struct rectangle range, kdNode* r, bool flag);
		kdNode* split(list<POI*> l, bool flag, combine *com);	//flag=true means to split x
		double dist(const point& p1, const point& p2);
		double Rad(double d);
		double dist_from_point_to_rect(point p, kdNode* node);
		double dist_from_point_to_rect(point p, double, double, double, double);
		bool point_in_range(const struct point& p, const struct rectangle& range);
		bool point_in_range(const struct point& p, const struct circle& range);
		
		//KNN global data
		struct Pair
		{
			POI* poi;
			double dist;
			Pair(POI* p, double d)
			{
				poi = p;
				dist = d;
			}
			friend bool operator<(const Pair& p1, const Pair& p2)
			{
				return p1.dist<p2.dist;
			}
		};
		point knn_p;
		int knn_k;
		priority_queue<Pair> knn_q;
		void KNNQuery(kdNode* r);

		//for KNNScan
		struct InversePair
		{
			POI* poi;
			double dist;
			InversePair(POI* p, double d)
			{
				poi = p;
				dist = d;
			}
			friend bool operator<(const InversePair& p1, const InversePair& p2)
			{
				return p1.dist>p2.dist;
			}
		};

		double dist_to_long(double, double);
		double dist_to_lat(double);

		void KNNQuery2_grid_scan(int, list<POI*>*);
		bool KNNQuery2_process(int, int, list<POI*>*);

	//only for testing and debugging
	public:
		void test(int cat);
		void bianli(kdNode* r);
};
