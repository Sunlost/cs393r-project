#include <utility>
#include <map>
#include <list>
#include <cmath>

using std::map;
using std::pair;
using std::list;

class Point {
public:

    Point() { x = 0.0; y = 0.0; }

    Point(float x, float y) {
        this->x = x;
        this->y = y;
    }

    float x;
    float y;
};

inline bool operator<(const Point& lhs, const Point& rhs) {
    if(lhs.x != rhs.x) return lhs.x < rhs.x;
    else return lhs.y < lhs.y;
}

inline bool operator==(const Point& lhs, const Point& rhs) {
    return (lhs.x == rhs.x) && (lhs.y == rhs.y);
}

inline bool operator!=(const Point& lhs, const Point& rhs) {
    return (lhs.x != rhs.x) || (lhs.y != rhs.y);
}

inline float norm_dist(Point a, Point b) {
    return sqrt(pow(a.x + b.x, 2) + pow(a.y + b.y, 2));
}



class Edge {
public:

    Edge(int a, int b, float dist){
        this->a = a;
        this->b = b;
        this->dist = dist;
    }

    int a;
    int b;
    float dist;
};

inline bool operator < (Edge const& lhs, Edge const& rhs) {
    return lhs.dist < rhs.dist;
}

class IncrCompare {
public:
    bool operator () (Edge* a, Edge* b) {
        return a->dist < b->dist;
    } 
};

class DecrCompare {
public:
    bool operator () (Edge* a, Edge* b) {
        return a->dist > b->dist;
    } 
};



class UnionFind {
    int *parent;
    int *rank;

public:

    UnionFind(uint64_t V){
        parent = new int[V];
        rank = new int[V];
        for(uint64_t i = 1; i <= V; i++) {
            parent[i] = i;
            rank[i] = 1;
        }
    };

    int find(int p) {
        while(p != parent[p]) {
            parent[p] = parent[parent[p]];
            p = parent[p];
        }
        return p;
    }

    void unite(int a, int b) {
        int pa = find(a);
        int pb = find(b);

        if(rank[pb] < rank[pa]) {
            int swap = pa;
            pa = pb;
            pb = swap;
        }

        parent[pa] = pb;
        rank[pb] += rank[pa];
    }
};

namespace coverage_planner {

void construct_global_coverage_path(map<pair<float, float>, list<pair<float, float> > > &edge_map,
                                    pair<float, float> starting_node, 
                                    list<pair<float, float>* > &output);

}