#include<iostream>
#include<cstdio>
#include<string>
#include "POI.h"

using namespace std;

const int CATEGORY = 71100;

void print_list(list<POI*>& res)
{
	int k = 0;
	for(list<POI*>::iterator ite = res.begin(); ite != res.end(); ++ite)
		cout<<++k<<": "<<(*ite)->id<<endl;
	cout<<"Total: "<<res.size()<<endl;
}

int main(int args, char** argv)
{
	Search engine;
	engine.IndexBuilding("input");
	engine.IndexBuilding2("input",10,10);
	engine.test(CATEGORY);

	list<POI*> res;

	cout<<"------------------Rectangle RangeQuery(KD)--------------------"<<endl;
	res = engine.RangeQuery(rectangle(point(121.44, 31.02), point(121.45, 31.04)), CATEGORY);
	print_list(res);
	
	cout<<"------------------Rectangle RangeQuery(GRID)--------------------"<<endl;
	res = engine.RangeQuery2(rectangle(point(121.44, 31.02), point(121.45, 31.04)), CATEGORY);
	print_list(res);

	cout<<"---------------------Brute Scan---------------------------"<<endl;
	res = engine.Scan(rectangle(point(121.44,31.02), point(121.45,31.04)), CATEGORY, "input");
	print_list(res);

	cout<<"----------------------Circle RangeQuery(KD)-------------------"<<endl;
	res = engine.RangeQuery(circle(point(121.411, 31.042), 200), CATEGORY);
	print_list(res);

	cout<<"----------------------Circle RangeQuery(GRID)-------------------"<<endl;
	res = engine.RangeQuery2(circle(point(121.411, 31.042), 200), CATEGORY);
	print_list(res);

	cout<<"----------------------Brute Scan--------------------------"<<endl;
	res = engine.Scan(circle(point(121.411, 31.042), 200), CATEGORY, "input");
	print_list(res);

	cout<<"-------------------KNN Query(KD)-----------------------------"<<endl;
	res = engine.KNNQuery(point(121.411, 31.042), CATEGORY, 10);
	print_list(res);

	cout<<"-------------------KNN Query(GRID)-----------------------------"<<endl;
	res = engine.KNNQuery2(point(121.411, 31.042), CATEGORY, 10);
	print_list(res);

	cout<<"-------------------Brute Scan-----------------------------"<<endl;
	res = engine.KNNScan(point(121.411, 31.042), CATEGORY, 10, "input");
	print_list(res);
}
