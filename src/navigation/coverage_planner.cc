#include "coverage_planner.h"

#include <algorithm>
#include <list>
#include <queue>
#include <set>
#include <cassert>
#include <iostream>

using std::pair;
using std::map;
using std::list;
using std::queue;
using std::set;
using std::priority_queue;
using std::vector;


// hand-written implementation of modified TSP by Sun O'Brien
// things to note:
    // 1. input graph does *not* need to be fully-connected
    // 2. resulting TSP *may* revisit nodes
    // 3. approximation is 1/2


// trim graph to one strongly connected component via bfs
void trim_graph(map<pair<float, float>, list<pair<float, float> > > edge_map, 
                map<Point, list<Point> > trimmed_map, pair<float, float> starting_node) {
    queue<Point> q;
    q.push(Point(starting_node.first, starting_node.second));

    set<Point> seen;
    
    while(q.size() > 0) {
        Point pop = q.front();
        q.pop();
        if(!seen.insert(pop).second) continue;

        list<pair<float, float> > adj = edge_map[pair<float, float>(pop.x, pop.y)];
        for(pair<float, float> adj_point : adj) {
            trimmed_map[pop].push_back(Point(adj_point.first, adj_point.second));
        }
    }

    return;
}



// all-pairs-shortest-path via floyd-warshall
void floyd_warshall(map<Point, list<Point> > trimmed_map, map<Point, int> index_map, 
                    map<int, Point> point_map, float **dist, int **prev) {

    int count = 0;
    for(auto it = trimmed_map.begin(); it != trimmed_map.end(); it++) {
        index_map[it->first] = count;
        point_map[count] = it->first;
        dist[count][count] = 0;
        prev[count][count] = 0;
        count++;
    }
    for(auto it = trimmed_map.begin(); it != trimmed_map.end(); it++) {
        Point u = it->first;
        int u_idx = index_map[u];
        for(auto v : it->second) {
            int v_idx = index_map[v];
            dist[u_idx][v_idx] = u.dist(v);
            prev[u_idx][v_idx] = u_idx;
        }
    }

    int V = index_map.size();
    for(int k = 0; k < V; k++) {
        for(int i = 0; i < V; i++) {
            for(int j = 0; j < V; j++) {
                if(dist[i][j] > dist[i][k] + dist[k][j]) {
                    dist[i][j] = dist[i][k] + dist[k][j];
                    prev[i][j] = prev[k][j];
                }
            }
        }
    }
}



// kruskal's MST algorithm via union-find
void build_mst(float **dist, uint64_t V, map<int, list<int> > mst) {
    priority_queue<Edge> pq;
    for(uint64_t i = 0; i < V; i++) {
        for(uint64_t j = 0; j < V; j++) {
            if(i == j) continue;
            pq.push(Edge(i, j, dist[i][j]));
        }
    }

    UnionFind uf(V);
    list<Edge> taken_edges;

    while(taken_edges.size() < (uint64_t) V - 1) {
        Edge edge = pq.top();
        pq.pop();
        if(uf.find(edge.a) != uf.find(edge.b)) {
            uf.unite(edge.a, edge.b);
            taken_edges.push_back(edge);
        }
    }

    // insert undirected edges into mst representation
    for(auto edge = taken_edges.begin(); edge != taken_edges.end(); edge++) {
        mst[edge->a].push_back(edge->b);
        mst[edge->b].push_back(edge->a);
    }

    return;
}



// build min-weight perfect matching via D. Vinkemeier and S. Hougardy's approximation algo.
void build_matching(map<int, list<int> > mst, float **dist, list<pair<int, int> > matching) {
    // find odd-degree vertices
    list<int> odd_list;
    for(auto it = mst.begin(); it != mst.end(); it++) {
        if(it->second.size() % 2 == 1) odd_list.push_back(it->first);
    }

    // by the handshaking lemma, there must be an even number of odd vertices in the graph

    // build min-weight perfect matching
        // optimal is O(|V|^2 * |E|) [see Blossom V by V. Kolmogorov (2009)]
        // so we run a greedy version instead [see "A linear-time approx..." by D. Vinkemeier and S. Hougardy (2003)]
        // provides a 1/2 max-weight approximation in O(m log n) time

    priority_queue<Edge, vector<Edge>, EdgeNonDecrComparator> pq;
    for(int i : odd_list) {
        for(int j : odd_list) {
            pq.push(Edge(i, j, dist[i][j] * -1));
        }
    }

    set<int> matched_vertices;
    while(!pq.empty() && matched_vertices.size() < odd_list.size()) {
        Edge edge = pq.top();
        pq.pop();
        if(matched_vertices.insert(edge.a).second) {
            if(matched_vertices.insert(edge.b).second) {
                // successful match
                matching.push_back(pair<int, int>(edge.a, edge.b));
                matching.push_back(pair<int, int>(edge.b, edge.a));
            } else {
                // a is unmatched but b is matched.
                matched_vertices.erase(edge.a);
            }
        }
    }

    assert(matched_vertices.size() == odd_list.size());

    return;
}



// build eulerian tour in graph via Hierholzer's algorithm
void build_eulerian_tour(map<int, list<int> > mst,
                         list<int> eulerian_tour) {
    list<int> cur_path;
    cur_path.push_back(mst.begin()->first); // start with an element; doesn't really matter which.

    map<int, list<int>::iterator> adj_remaining;
    for(auto it = mst.begin(); it != mst.end(); it++) {
        adj_remaining[it->first] = it->second.begin();
    }

    while(cur_path.size() > 0) {
        int cur = cur_path.back();
        if(adj_remaining[cur] != mst[cur].end()) {
            int next = *adj_remaining[cur];
            adj_remaining[cur]++;
            cur_path.push_back(next);
        } else {
            eulerian_tour.push_back(cur);
            cur_path.pop_back();
        }
    }

    return;
}



float build_hamiltonian_path(list<int> eulerian_tour, list<int> hamiltonian_path, float **dist) {
    map<int, bool> visited;
    for(uint64_t i = 0; i < eulerian_tour.size(); i++) {
        visited[i] = false;
    }

    float total_dist = 0.0;
    visited[eulerian_tour.front()] = true;
    list<int>::iterator cur = eulerian_tour.begin();
    list<int>::iterator next = eulerian_tour.begin()++;

    while(next != eulerian_tour.end()) {
        if(visited[*next]) {
            next = eulerian_tour.erase(next);
        } else {
            total_dist += dist[*cur][*next];
            cur = next;
            visited[*cur] = true;
            next = cur++;
        }
    }

    return total_dist + dist[*cur][*next];
}



void construct_final_path(list<pair<float, float> > output, int **prev, 
                          list<int> hamiltonian_path, pair<float, float> starting_node,
                          map<int, Point> point_map, map<Point, int> index_map) {

    int start_idx = index_map[Point(starting_node.first, starting_node.second)];
    list<int>::iterator it = std::find(hamiltonian_path.begin(), hamiltonian_path.end(), start_idx);
    assert(it != hamiltonian_path.end());

    output.push_back(pair<float, float>(starting_node.first, starting_node.second)); // deep copy

    int u_idx, v_idx;
    list<pair<float, float> > subpath;

    // go from our custom starting point to the end
    while(true) {
        u_idx = *it;
        it++;
        if(it == hamiltonian_path.end()) break;

        v_idx = *it;
        subpath.clear();
        subpath.push_front(pair<float, float>(point_map[v_idx].x, point_map[v_idx].y));
        while(u_idx != v_idx) {
            v_idx = prev[u_idx][v_idx];
            subpath.push_front(pair<float, float>(point_map[v_idx].x, point_map[v_idx].y));
        }
        output.splice(output.end(), subpath);
    }

    // handle loop from end to start
    v_idx = *hamiltonian_path.begin();
    subpath.clear();
    subpath.push_front(pair<float, float>(point_map[v_idx].x, point_map[v_idx].y));
    while(u_idx != v_idx) {
        v_idx = prev[u_idx][v_idx];
        subpath.push_front(pair<float, float>(point_map[v_idx].x, point_map[v_idx].y));
    }
    output.splice(output.end(), subpath);

    // aaaand now go from start to custom starting point
    list<int>::iterator stop = std::find(hamiltonian_path.begin(), hamiltonian_path.end(), start_idx);
    it = hamiltonian_path.begin();
    while(true) {
        if(it == stop) break;
        u_idx = *it;
        it++;
        v_idx = *it;
        subpath.clear();
        subpath.push_front(pair<float, float>(point_map[v_idx].x, point_map[v_idx].y));
        while(u_idx != v_idx) {
            v_idx = prev[u_idx][v_idx];
            subpath.push_front(pair<float, float>(point_map[v_idx].x, point_map[v_idx].y));
        }
        output.splice(output.end(), subpath);
    }

}


void coverage_planner::construct_global_coverage_path(map<pair<float, float>, list<pair<float, float> > > edge_map,
                                    pair<float, float> starting_node, list<pair<float, float> > output) {
    // 1. trim map down to a single connected component C.
    map<Point, list<Point> > trimmed_map;
    trim_graph(edge_map, trimmed_map, starting_node);

    // 2. assign vertex IDs. build all-pairs-shortest-paths solution via Floyd-Warshall.
    map<Point, int> index_map;
    map<int, Point> point_map;
    float *dist[trimmed_map.size()];
    int *prev[trimmed_map.size()];
    for(uint64_t i = 0; i < trimmed_map.size(); i++) {
        dist[i] = new float[trimmed_map.size()];
        prev[i] = new int[trimmed_map.size()];
    }
    floyd_warshall(trimmed_map, index_map, point_map, dist, prev);

    // 3. build a minimum spanning tree T in C
    map<int, list<int> > mst;
    build_mst(dist, index_map.size(), mst);

    // 4. build a perfect matching M of odd-degree vertices in C 
    list<pair<int, int> > matching;
    build_matching(mst, dist, matching);

    // 5. combine T and M to make multigraph T'
    for(pair<int, int> pair : matching) {
        list<int> adj_list = mst[pair.first];

        if(std::find(adj_list.begin(), adj_list.end(), pair.second) != adj_list.end()) {
            adj_list.push_back(pair.second);
            mst[pair.first] = adj_list;
        }
    }
    assert(mst.size() != 0);
    assert(mst.size() == index_map.size());

    // 6. identify a eulerian tour E in T'
    list<int> eulerian_tour;
    build_eulerian_tour(mst, eulerian_tour);
    assert(eulerian_tour.size() == mst.size());

    // 7. reduce E to a hamiltonian tour H
    list<int> hamiltonian_path;
    build_hamiltonian_path(eulerian_tour, hamiltonian_path, dist);

    // 8. reconstruct paths via floyd-warshall prev list
    construct_final_path(output, prev, hamiltonian_path, starting_node, point_map, index_map);

    return;
}

void coverage_planner::pineapple() {
    std::cout << "test" << std::endl;
}