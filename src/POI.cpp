#include "POI.h"
#include<cmath>

using namespace std;

bool cmpx(const POI* obj1, const POI* obj2)
{
	if(obj1->p.x<obj2->p.x) return true;
	else return false;
}

bool cmpy(const POI* obj1, const POI* obj2)
{
	if(obj1->p.y<obj2->p.y) return true;
	else return false;
}

bool Search::point_in_range(const struct point& p, const struct rectangle& range)
{
	if(p.x>=range.upperLeft.x&&p.x<=range.bottomRight.x)
	{
		if(p.y>=range.upperLeft.y&&p.y<=range.bottomRight.y)
			return true;
	}
	return false;
}

bool Search::point_in_range(const struct point& p, const struct circle& range)
{
	if(dist(p, range.center)<=range.radius)
		return true;
	return false;
}

double Search::dist_from_point_to_rect(point p, double minx, double miny, double maxx, double maxy)
{
	if(p.x<minx)
	{
		if(p.y<miny) return dist(point(minx, miny), p);
		else if(p.y>maxy) return dist(point(minx, maxy), p);
		else return dist(point(minx, p.y), p);
	}
	else if(p.x>maxx)
	{
		if(p.y<miny) return dist(point(maxx, miny), p);
		else if(p.y>maxy) return dist(point(maxx, maxy), p);
		else return dist(point(maxx, p.y), p);
	}
	else
	{
		if(p.y<miny) return dist(point(p.x, miny), p);
		else if(p.y>maxy) return dist(point(p.x, maxy), p);
		else return 0;
	}
}

double Search::dist_from_point_to_rect(point p, kdNode* node)
{
	double minx = node->minx, maxx = node->maxx;
	double miny = node->miny, maxy = node->maxy;

	return dist_from_point_to_rect(p, minx, miny, maxx, maxy);
}

double Search::Rad(double d)
{
	return d * PI / 180.0;
}

double Search::dist(const point& p1, const point& p2)
{
	double lat1 = p1.y, lat2 = p2.y;
	double lon1 = p1.x, lon2 = p2.x;

	double radLat1 = Rad(lat1);
	double radLat2 = Rad(lat2);
        double delta_lon = Rad(lon2 - lon1);
        double top_1 = cos(radLat2) * sin(delta_lon);
        double top_2 = cos(radLat1) * sin(radLat2) - sin(radLat1) * cos(radLat2) * cos(delta_lon);
        double top = sqrt(top_1 * top_1 + top_2 * top_2);
        double bottom = sin(radLat1) * sin(radLat2) + cos(radLat1) * cos(radLat2) * cos(delta_lon);
        double delta_sigma = atan2(top, bottom);
        double distance = delta_sigma * 6378137.0;
        return distance;
}

double Search::dist_to_lat(double x)
{
	double tmp =  (2 * PI * 6378137.0) / 360;
	return x / tmp;
}

double Search::dist_to_long(double x, double lat)
{
	double tmp = (2 * PI * 6378137.0 * cos(Rad(lat))) / 360;
	return x / tmp;
}

void Search::IndexBuilding(string file_name)
{
	fin.open(file_name.c_str());

	string id;
	int cat;
	double x,y;

	//new table mapping each category to a distinct kd-tree
	table = map<int, list<POI*>* >();
	map<int, combine*> record;

	//classify all POIs
	while(fin>>id>>cat>>x>>y)
	{
		POI* poi = new POI(id, cat, x, y);

		map<int, list<POI*>* >::iterator ite;
		ite = table.find(cat);
		if(ite==table.end())
		{
			list<POI*>* l = new list<POI*>();
			l->push_back(poi);
			table.insert(pair<int, list<POI*>* >(cat, l));

			record.insert(pair<int, combine*>(cat, new combine(x,y)));
		}
		else
		{
			list<POI*>* l = ite->second;
			l->push_back(poi);
			combine* com = record.find(cat)->second;
			
			if(x>com->maxx) com->maxx = x;
			if(x<com->minx) com->minx = x;
			if(y<com->miny) com->miny = y;
			if(y>com->maxy) com->maxy = y;
		}
	}

	fin.close();

	//set up all kd-tree's
	trees = map<int, kdNode* >();

	map<int, list<POI*>* >::iterator ite;
	
	for(ite=table.begin();ite!=table.end();++ite)
	{
		list<POI*>* l = ite->second;

		combine *com = record.find(ite->first)->second;
		kdNode* root = split(*l, true, com);
		root->minx = com->minx;
		root->maxx = com->maxx;
		root->miny = com->miny;
		root->maxy = com->maxy;
		
		trees.insert(pair<int, kdNode*>(ite->first, root));
	}
}

int Search::abs(int x)
{
	if(x<0) return -x;
	return x;
}

kdNode* Search::split(list<POI*> l, bool flag, combine *com)
{
	if(l.size()<=MaxPOIPerNode) 
		return new kdNode(l);
	else
	{
		if(flag) l.sort(cmpx);
		else l.sort(cmpy);
		
		int k=0;
		list<POI*> newList;
		while(abs(l.size()-k)>1)
		{
			newList.push_back(l.front());
			l.pop_front();
			++k;
		}
	
		POI* split_poi = l.front();
		l.pop_front();
	
		double minx, miny, maxx, maxy;
		minx = com->minx; maxx = com->maxx; 
		miny = com->miny; maxy = com->maxy;
		if(flag) maxx = split_poi->p.x;
		else maxy = split_poi->p.y;
		kdNode* le = split(newList, !flag, new combine(minx, maxx, miny, maxy));
		le->minx = minx; le->maxx = maxx;
		le->miny = miny; le->maxy = maxy;

		minx = com->minx; maxx = com->maxx; 
		miny = com->miny; maxy = com->maxy;
		if(flag) minx = split_poi->p.x;
		else miny = split_poi->p.y;
		kdNode* r = split(l, !flag, new combine(minx, maxx, miny, maxy));
		r->minx = minx;	r->maxx = maxx;
		r->miny = miny; r->maxy = maxy;
		
		return new kdNode(flag, split_poi, le, r);
	}
}

list<POI*> Search::RangeQuery(struct rectangle range, int cat)
{
	tot_running_time = 0;

	map<int, kdNode*>::iterator ite = trees.find(cat);
	if(ite==trees.end()) 
	{
		list<POI*> res;
		return res;
	}
	
	kdNode* root = ite->second;
	
	list<POI*> res = RangeQuery(range, root, true);
	
	cout<<"TOTAL_RUNING_TIME: "<<tot_running_time<<endl;

	return res;
}

list<POI*> Search::RangeQuery(struct rectangle range, kdNode* r, bool flag)
{
	++tot_running_time;

	list<POI*> res;
	if(r->isLeaf)
	{
		list<POI*> l = r->data;
		for(list<POI*>::iterator i=l.begin();i!=l.end();++i)
		{
			++tot_running_time;
			if(point_in_range((*i)->p, range))
				res.push_back(*i);
		}
	}
	else
	{
		struct point upperLeft = range.upperLeft;
		struct point bottomRight = range.bottomRight;
		if(flag)
		{
			if(r->split->p.x>upperLeft.x&&r->split->p.x<bottomRight.x)
			{
				res = RangeQuery(rectangle(upperLeft, point(r->split->p.x, bottomRight.y)), r->left, false);
				list<POI*> tmp = RangeQuery(rectangle(point(r->split->p.x, upperLeft.y), bottomRight), r->right, false);
				for(list<POI*>::iterator ite=tmp.begin();ite!=tmp.end();++ite)
					res.push_back(*ite);
			}
			else if(r->split->p.x<=upperLeft.x)
			{
				res = RangeQuery(range, r->right, false);
			}
			else
			{
				res = RangeQuery(range, r->left, false);
			}
		}
		else
		{
			if(r->split->p.y>upperLeft.y&&r->split->p.y<bottomRight.y)
			{
				res = RangeQuery(rectangle(upperLeft, point(bottomRight.x, r->split->p.y)), r->left, true);
				list<POI*> tmp = RangeQuery(rectangle(point(upperLeft.x, r->split->p.y), bottomRight), r->right, true);
				for(list<POI*>::iterator ite=tmp.begin();ite!=tmp.end();++ite)
					res.push_back(*ite);
			}
			else if(r->split->p.y<=upperLeft.y)
			{
				res = RangeQuery(range, r->right, true);
			}
			else
			{
				res = RangeQuery(range, r->left, true);
			}
		}

		if(point_in_range(r->split->p, range))
			res.push_back(r->split);
	}
	return res;
}

list<POI*> Search::RangeQuery(struct circle range, int cat)
{
	tot_running_time = 0;

	double lat_delta = dist_to_lat(range.radius);
	double long_delta = dist_to_long(range.radius, range.center.y);

	list<POI*> poss = RangeQuery(rectangle(point(range.center.x-long_delta, range.center.y-lat_delta), point(range.center.x+long_delta, range.center.y+lat_delta)), cat);
	list<POI*> res;
	for(list<POI*>::iterator ite=poss.begin();ite!=poss.end();++ite)
	{
		++tot_running_time;
		if(dist(range.center, (*ite)->p) < range.radius)
		{
			res.push_back(*ite);
		}
	}

	cout<<"TOTAL_RUNNING_TIME: "<<tot_running_time<<endl;

	return res;
}

list<POI*> Search::KNNQuery(struct point p, int cat, int index)
{
	knn_q = priority_queue<Pair>();
	knn_k = index; 
	knn_p = p;

	tot_running_time = 0;

	map<int, kdNode*>::iterator ite = trees.find(cat);
	list<POI*> res;
	if(ite==trees.end()) return res;
	kdNode* root = ite->second;
	KNNQuery(root);
	int k=knn_k;
	while(k>0&&!knn_q.empty())
	{
		res.push_back(knn_q.top().poi);
		//cout<<knn_q.top().poi->id<<": "<<knn_q.top().dist<<endl;
		knn_q.pop();
		--k;
	}
	res.reverse();

	cout<<"TOTAL_RUNNING_TIME: "<<tot_running_time<<endl;

	return res;
}

void Search::KNNQuery(kdNode* r)
{
	++tot_running_time;

//	cout<<"!!!!"<<r->minx<<" "<<r->maxx<<"; "<<r->miny<<" "<<r->maxy<<endl;
	if(r->isLeaf)
	{
		for(list<POI*>::iterator ite=r->data.begin();ite!=r->data.end();++ite)
		{
			++tot_running_time;

			double new_dist = dist((*ite)->p, knn_p);
			knn_q.push(Pair(*ite, new_dist));
			if(knn_q.size()>knn_k) knn_q.pop();
		}
	}
	else
	{
		double new_dist = dist(knn_p, r->split->p);
		//cout<<"("<<knn_p.x<<", "<<knn_p.y<<")  ";
		//cout<<"("<<r->split->p.x<<", "<<r->split->p.y<<")   ";
		//cout<<r->split->id<<": "<<new_dist<<" "<<(knn_q.empty()? -1:knn_q.top().dist)<<endl;
		
		++tot_running_time;

		knn_q.push(Pair(r->split, new_dist));
		if(knn_q.size()>knn_k) knn_q.pop();
		
		double left_dist = dist_from_point_to_rect(knn_p, r->left);
		double right_dist = dist_from_point_to_rect(knn_p, r->right);
		
		double max_dist = knn_q.top().dist;
		if(left_dist<right_dist)
		{
			if(left_dist<max_dist||knn_q.size()<knn_k) KNNQuery(r->left);
			max_dist = knn_q.top().dist;
			if(right_dist<max_dist||knn_q.size()<knn_k) KNNQuery(r->right);
		}
		else
		{
			if(right_dist<max_dist||knn_q.size()<knn_k) KNNQuery(r->right);
			max_dist = knn_q.top().dist;
			if(left_dist<max_dist||knn_q.size()<knn_k) KNNQuery(r->left);
		}
	}
}

void Search::test(int cat)
{
	map<int, list<POI*>* >::iterator ite = table.find(cat);
	if(ite==table.end())
	{
		cout<<"Sorry, not found!"<<endl;
	}
	else
	{
		cout<<"OK, here they are: "<<endl;
		list<POI*>* l = ite->second;
		cout<<l->size()<<endl;
		for(list<POI*>::iterator i=l->begin();i!=l->end();++i)
		{
			cout<<"\t"<<(*i)->id<<"\t"<<(*i)->p.x<<"\t"<<(*i)->p.y<<endl;
		}
		map<int, kdNode*>::iterator it = trees.find(cat);
		cout<<"Following is the tree!"<<endl;
		kdNode* root = it->second;
		bianli(root);
	}
}

void Search::bianli(kdNode* r)
{
	cout<<"{";
	if(r->isLeaf) 
	{
		for(list<POI*>::iterator ite=r->data.begin();ite!=r->data.end();++ite)
			cout<<(*ite)->id<<" ";
		cout<<"}"<<endl;
	}
	else 
	{
		cout<<r->split->id<<":";
		bianli(r->left);
		bianli(r->right);
		cout<<"}";
	}
}

list<POI*> Search::Scan(struct rectangle range, int cat, string file)
{
	ifstream fin2(file.c_str());

	list<POI*> l;

	tot_running_time = 0;

	string id;
	int c;
	double x, y;

	while(fin2>>id>>c>>x>>y)
	{
		if(c==cat) ++tot_running_time;

		if(c==cat&&point_in_range(point(x,y), range))
		{
			POI* poi = new POI(id, cat, x, y);
			l.push_back(poi);
		}
	}

	fin2.close();

	cout<<"TOTAL_RUNNING_TIME: "<<tot_running_time<<endl;

	return l;
}

list<POI*> Search::Scan(struct circle range, int cat, string file)
{
	ifstream fin2(file.c_str());

	tot_running_time = 0;

	list<POI*> l;

	string id;
	int c;
	double x, y;

	while(fin2>>id>>c>>x>>y)
	{
		if(c==cat) ++tot_running_time;

		if(c==cat&&point_in_range(point(x,y), range))
		{
			POI* poi = new POI(id, cat, x, y);
			l.push_back(poi);
		}
	}

	cout<<"TOTAL_RUNNING_TIME: "<<tot_running_time<<endl;

	fin2.close();

	return l;
}

list<POI*> Search::KNNScan(struct point p, int cat, int index, string file)
{
	ifstream fin2(file.c_str());

	list<POI*> l;

	tot_running_time = 0;

	string id;
	int c;
	double x, y;

	priority_queue<InversePair> q;

	while(fin2>>id>>c>>x>>y)
	{
		if(c==cat)
		{
			++tot_running_time;

			POI* poi = new POI(id, c, x, y);
			q.push(InversePair(poi, dist(p, point(x, y))));
		}
	}

	while(index>0&&!q.empty())
	{
		l.push_back(q.top().poi);
		q.pop();
		--index;
	}

	fin2.close();

	cout<<"TOTAL_RUNNING_TIME: "<<tot_running_time<<endl;

	return l;
}

void Search::IndexBuilding2(string file_name, int xtot, int ytot)
{
	x_tot = xtot;
	y_tot = ytot;
	IndexBuilding2(file_name);
}

void Search::IndexBuilding2(string file_name)
{
	fin.open(file_name.c_str());

	string id;
	int cat;
	double x,y;

	table2 = map<int, list<POI*>* >();

	int NOT_READY = 1;

	//classify all POIs
	while(fin>>id>>cat>>x>>y)
	{
		POI* poi = new POI(id, cat, x, y);
			
		if(NOT_READY) 
		{
			x_max = x_min = x;
			y_max = y_min = y;
			NOT_READY = 0;
		}

		if(x>x_max) x_max = x;
		if(x<x_min) x_min = x;
		if(y<y_min) y_min = y;
		if(y>y_max) y_max = y;

		map<int, list<POI*>* >::iterator ite;
		ite = table2.find(cat);
		if(ite==table2.end())
		{
			list<POI*>* l = new list<POI*>();
			l->push_back(poi);
			table2.insert(pair<int, list<POI*>* >(cat, l));
		}
		else
		{
			list<POI*>* l = ite->second;
			l->push_back(poi);
		}
	}

	fin.close();

	x_step = (x_max - x_min)/x_tot;
	y_step = (y_max - y_min)/y_tot;

	//set up all vects
	vects = map<int, vector<list<POI*>*>*>();
	
	map<int, list<POI*>* >::iterator ite;

	for(ite=table2.begin(); ite!=table2.end(); ++ite)
	{
		list<POI*>* l = ite->second;
		vector<list<POI*>*>* vect = new vector<list<POI*>*>(x_tot*y_tot);
		vects.insert(pair<int, vector<list<POI*>*>*>(ite->first, vect));
		for(list<POI*>::iterator i=l->begin(); i!=l->end(); ++i)
		{
			int index = grid_number((*i)->p.x, (*i)->p.y);
			if((*vect)[index]==0)
			{
				list<POI*>* l = new list<POI*>();
				l->push_back(*i);
				(*vect)[index] = l;
			}
			else
			{
				(*vect)[index]->push_back(*i);
			}
		}
	}
}

list<POI*> Search::KNNQuery2(struct point p, int cat, int index)
{
	list<POI*> res = list<POI*>();

	tot_running_time = 0;

	knn_q = priority_queue<Pair>();
	knn_k = index; 
	knn_p = p;

	map<int, vector<list<POI*>*>*>::iterator ite = vects.find(cat);

	if(ite!=vects.end())
	{
		vector<list<POI*>*>* vect = ite->second;

		if(p.x>x_max||p.x<x_min||p.y>y_max||p.y<y_min) 
		{
			cout<<"ERROR: The query point is not in the range!"<<endl;
			return res;
		}

		int g = grid_number(p.x, p.y);
		
		int x = g / y_tot, y = g % y_tot, deep = 0;

		//cout<<x<<" "<<y<<endl;

		KNNQuery2_grid_scan(g, (*vect)[g]);
		
		bool flag = true;
		while(flag)
		{
			++tot_running_time;

			//cout<<deep+1<<endl;
			++deep; flag = false;
			double min_dist = -1;

			int i, j;

			j = y + deep;
			if(j < y_tot)
			{
				for(int i = max(x-deep, 0); i <= min(x+deep, x_tot-1); ++i)
				{
					++tot_running_time;
					flag |= KNNQuery2_process(i, j, (*vect)[i * y_tot + j]);
				}
			}
			
			j = y - deep;
			if(j >= 0)
			{
				for(int i = max(x-deep, 0); i <= min(x+deep, x_tot-1); ++i)
				{
					++tot_running_time;
					flag |= KNNQuery2_process(i, j, (*vect)[i * y_tot + j]);
				}
			}
			
			i = x + deep;
			if(i < x_tot)
			{
				for(int j = max(y-deep, 0); j <= min(y+deep, y_tot-1); ++j)
				{
					++tot_running_time;
					flag |= KNNQuery2_process(i, j, (*vect)[i * y_tot + j]);
				}
			}

			i = x - deep;
			if(i >= 0)
			{
				for(int j = max(y-deep, 0); j <= min(y+deep, y_tot-1); ++j)
				{
					++tot_running_time;
					flag |= KNNQuery2_process(i, j, (*vect)[i * y_tot + j]);
				}
			}
		}
	}

	while(!knn_q.empty())
	{
		POI* poi = knn_q.top().poi;
		//cout<<poi->id<<": "<<knn_q.top().dist<<endl;
		res.push_front(poi);
		knn_q.pop();
	}

	cout<<"TOTAL_RUNNING_TIME: "<<tot_running_time<<endl;

	return res;
}

bool Search::KNNQuery2_process(int i, int j, list<POI*>* l)
{
	double minx = x_min + i * x_step, miny = y_min + j * y_step;
	double maxx = x_min + (i+1) * x_step, maxy = y_min + (j+1) * y_step;

	if(knn_q.size() < knn_k || knn_q.top().dist > dist_from_point_to_rect(knn_p, minx, miny, maxx, maxy))
	{
		KNNQuery2_grid_scan(i*y_tot+j, l);
		return true;
	}
	return false;
}

void Search::KNNQuery2_grid_scan(int g, list<POI*>* l)
{
	if(l == 0) return;

	for(list<POI*>::iterator ite=l->begin(); ite!=l->end(); ++ite)
	{
		++tot_running_time;

		double dis = dist(knn_p, (*ite)->p);
		knn_q.push(Pair(*ite, dis));
		if(knn_q.size()>knn_k) knn_q.pop();
	}
}

list<POI*> Search::RangeQuery2(struct rectangle range, int cat)
{
	list<POI*> res = list<POI*>();

	range = check_validity(range);

	tot_running_time = 0;

	map<int, vector<list<POI*>*>*>::iterator ite = vects.find(cat);
	
	if(ite!=vects.end())
	{
		vector<list<POI*>*>* vect = ite->second;

		int index1 = grid_number(range.upperLeft.x, range.upperLeft.y);
		int index2 = grid_number(range.bottomRight.x, range.bottomRight.y);
		int gx1 = index1 / y_tot, gy1 = index1 % y_tot;
		int gx2 = index2 / y_tot, gy2 = index2 % y_tot;

		for(int i=gx1; i<=gx2; ++i)
			for(int j=gy1; j<=gy2; ++j)
			{
				int index = i*y_tot + j;
				list<POI*>* l = (*vect)[index];
				
				++tot_running_time;

				if(l==0) continue;
				for(list<POI*>::iterator ddq = l->begin(); ddq!= l->end(); ++ddq)
				{
					++tot_running_time;

					if(point_in_range((*ddq)->p, range))
					{
						res.push_back(*ddq);
					}
				}
			}
	}

	cout<<"TOTAL_RUNNING_TIME: "<<tot_running_time<<endl;

	return res;
}

list<POI*> Search::RangeQuery2(struct circle range, int cat)
{
	list<POI*> res;

	tot_running_time = 0;

	double lat_delta = dist_to_lat(range.radius);
	double long_delta = dist_to_long(range.radius, range.center.y);
	
	struct point p1(range.center.x - long_delta, range.center.y - lat_delta);
	struct point p2(range.center.x + long_delta, range.center.y + lat_delta);
	struct rectangle range2(p1, p2);

	//cout<<"("<<range2.upperLeft.x<<", "<<range2.upperLeft.y<<") -> ("<<range2.bottomRight.x<<", "<<range2.bottomRight.y<<")"<<endl;
	range2 = check_validity(range2);
	//cout<<"("<<range2.upperLeft.x<<", "<<range2.upperLeft.y<<") -> ("<<range2.bottomRight.x<<", "<<range2.bottomRight.y<<")"<<endl;

	map<int, vector<list<POI*>*>*>::iterator ite = vects.find(cat);
	if(ite!=vects.end())
	{
		vector<list<POI*>*>* vect = ite->second;
		
		int index1 = grid_number(range2.upperLeft.x, range2.upperLeft.y);
		int index2 = grid_number(range2.bottomRight.x, range2.bottomRight.y);

		int gx1 = index1 / y_tot, gy1 = index1 % y_tot;
		int gx2 = index2 / y_tot, gy2 = index2 % y_tot;

		//int a[100][100]={0};

		for(int i=gx1; i<=gx2; ++i)
			for(int j=gy1; j<=gy2; ++j)
			{
				++tot_running_time;

				int index = i*y_tot + j;

				if(!grid_in_circle(index, range)) continue;
		
				list<POI*>* l = (*vect)[index];

		//		a[i][j]=1;

				if(l==0) continue;

				for(list<POI*>::iterator ddq = l->begin(); ddq!= l->end(); ++ddq)
				{
					++tot_running_time;

					if(point_in_range((*ddq)->p, range))
					{
						res.push_back(*ddq);
					}
				}
			}
		
		/*for(int i=gx1;i<=gx2;++i)
		{
			for(int j=gy1;j<=gy2;++j)
				cout<<a[i][j]<<" ";
			cout<<endl;
		}*/
	}

	cout<<"TOTAL_RUNNING_TIME: "<<tot_running_time<<endl;

	return res;
}

bool Search::grid_in_circle(int index, circle range)
{
	int x = index / y_tot, y = index % y_tot;
	
	double minx = x * x_step + x_min;
	double maxx = (x + 1) * x_step + x_min;
	double miny = y * y_step + y_min;
	double maxy = (y + 1) * y_step + y_min;

	if(dist_from_point_to_rect(range.center, minx, miny, maxx, maxy) < range.radius)
		return true;
	return false;
}

int Search::grid_number(double x, double y)
{
	int xx, yy;
	xx = (x - x_min) / x_step; if(xx >= x_tot) xx = x_tot - 1;
	yy = (y - y_min) / y_step; if(yy >= y_tot) yy = y_tot - 1;
	return xx * y_tot + yy;
}

rectangle Search::check_validity(const rectangle& range)
{
	double minx = range.upperLeft.x, miny = range.upperLeft.y; 
	double maxx = range.bottomRight.x, maxy = range.bottomRight.y;
	
	double tmp;

	if(minx > maxx) tmp = minx, minx = maxx, maxx = tmp;
	if(miny > maxy) tmp = miny, miny = maxy, maxy = tmp;

	if(minx < x_min) minx = x_min; if(minx > x_max) minx = x_max;
	if(miny < y_min) miny = y_min; if(miny > y_max) miny = y_max;
	if(maxx < x_min) maxx = x_min; if(maxx > x_max) maxx = x_max;
	if(maxy < y_min) maxy = y_min; if(maxy > y_max) maxy = y_max;

	return rectangle(point(minx, miny), point(maxx, maxy));
}
