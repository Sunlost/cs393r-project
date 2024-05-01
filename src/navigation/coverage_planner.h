#include <utility>
#include <map>
#include <list>
#include <cmath>

using std::map;
using std::pair;
using std::list;

class Point {
public:

    Point(){};

    Point(float x, float y) {
        this->x = x;
        this->y = y;
    };

    inline bool operator<(const Point& other) const {
        if(this->x == other.x) return this->x < other.x;
        else return this->y < other.y;
    }

    inline bool operator==(const Point& other) const {
        return (this->x == other.x) && (this->y == other.y);
    }

    inline bool operator!=(const Point& other) const {
        return !(*this == other);
    }

    float dist(Point other) {
        return sqrt(pow(this->x + other.x, 2) + pow(this->y + other.y, 2));
    }

    float x;
    float y;
};



class Edge {
public:

    Edge(){};

    Edge(int a, int b, float dist){
        this->a = a;
        this->b = b;
    };

    inline bool operator<(const Edge& other) const {
        return this->dist < other.dist;
    }

    int a;
    int b;
    float dist;
};

class EdgeNonDecrComparator {
public:
    bool operator() (Edge a, Edge b) {
        return a.dist > b.dist;
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
                                    list<pair<float, float> > &output);

void pineapple();

}