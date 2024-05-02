#include "coverage_planner.h"

#include <algorithm>
#include <limits>
#include <list>
#include <queue>
#include <set>
#include <cassert>
#include <iostream>
#include <cstring>

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
void trim_graph(map<pair<float, float>, list<pair<float, float> > > &edge_map, 
                map<Point, list<Point>* > &trimmed_map, pair<float, float> starting_node) {
    queue<Point> q;
    q.push(Point(starting_node.first, starting_node.second));

    set<Point> seen;
    
    while(q.size() > 0) {
        Point pop = q.front();
        q.pop();
        if(!seen.insert(pop).second) continue;

        list<pair<float, float> > adj = edge_map[pair<float, float>(pop.x, pop.y)];
        for(pair<float, float> adj_point : adj) {
            if(trimmed_map[pop] == nullptr) trimmed_map[pop] = new list<Point>; 
            trimmed_map[pop]->push_back(Point(adj_point.first, adj_point.second));
            q.push(Point(adj_point.first, adj_point.second));
        }
    }

    return;
}



int assign_vertex_ids(map<int, Point> &point_map, map<Point, int> &index_map, 
                       map<Point, list<Point>* > trimmed_map) {

    uint64_t count = 0;
    for(map<Point, list<Point>* >::iterator it = trimmed_map.begin(); it != trimmed_map.end(); it++) {
        if(index_map.find(it->first) == index_map.end()) {
            count++;
            index_map[it->first] = count;
            point_map[count] = it->first;
        }
        for(list<Point>::iterator inner_it = it->second->begin(); inner_it != it->second->end(); inner_it++) {
            if(index_map.find(*inner_it) != index_map.end()) continue;
            count++;
            index_map[*inner_it] = count;
            point_map[count] = *inner_it;
        }
    }
    return count;
}



// all-pairs-shortest-path via floyd-warshall
void floyd_warshall(map<Point, list<Point>* > &trimmed_map, map<Point, int> &index_map, 
                    map<int, Point> &point_map, vector<vector<float>* > &dist, 
                    vector<vector<int>* > &prev, uint64_t last_id) {

    for(map<Point, list<Point>* >::iterator it = trimmed_map.begin(); it != trimmed_map.end(); it++) {
        Point u = it->first;
        int u_idx = index_map[u];
        for(list<Point>::iterator inner_it = it->second->begin(); inner_it != it->second->end(); inner_it++) {
            Point v = *inner_it;
            int v_idx = index_map[v];

            if(dist[u_idx] == nullptr) dist[u_idx] = new vector<float>;
            (*dist[u_idx])[v_idx] = norm_dist(u, v);
            if((*dist[u_idx])[v_idx] == 0) (*dist[u_idx])[v_idx] = 0.001;

            if(prev[u_idx] == nullptr) prev[u_idx] = new vector<int>;
            (*prev[u_idx])[v_idx] = u_idx;
            assert((*prev[u_idx])[v_idx] != -1);
       }
    }

    for(map<Point, list<Point>* >::iterator it = trimmed_map.begin(); it != trimmed_map.end(); it++) {
        Point v = it->first;
        int v_idx = index_map[v];
        assert(dist[v_idx] != nullptr && prev[v_idx] != nullptr);
        (*dist[v_idx])[v_idx] = 0;
        (*prev[v_idx])[v_idx] = v_idx;
    }

    for(uint64_t k = 1; k <= last_id; k++) {
        if(k % 200 == 1) std::cout << "      >> floyd-warshall k = " << k << std::endl; 
        for(uint64_t i = 1; i <= last_id; i++) {
            for(uint64_t j = 1; j <= last_id; j++) {
                if((*dist[i])[j] > (*dist[i])[k] + (*dist[k])[j]) {
                    (*dist[i])[j] = (*dist[i])[k] + (*dist[k])[j];
                    (*prev[i])[j] = (*prev[k])[j];
                }
            }
        }
    }
}



// kruskal's MST algorithm via union-find
void build_mst(vector<vector<float>* > &dist, uint64_t V, map<int, list<int>* > &mst) {
    vector<Edge*> pq;
    for(uint64_t i = 1; i <= V; i++) {
        for(uint64_t j = 1; j <= V; j++) {
            if(i == j) continue;
            Edge* edge = new Edge(i, j, (*dist[i])[j]);
            pq.push_back(edge);
        }
    }
    
    std::sort(pq.begin(), pq.end(), IncrCompare());

    UnionFind uf(V);
    list<Edge*> taken_edges;

    for(vector<Edge*>::iterator it = pq.begin(); it != pq.end(); it++) {
        if(taken_edges.size() >= V - 1) break;
        Edge* edge = *it;
        if(uf.find(edge->a) != uf.find(edge->b)) {
            uf.unite(edge->a, edge->b);
            taken_edges.push_back(edge);
        }
    }

    // MST of V vertices will have V-1 edges
    assert(taken_edges.size() == V - 1);

    // insert undirected edges into mst representation
    for(list<Edge*>::iterator it = taken_edges.begin(); it != taken_edges.end(); it++) {
        Edge* edge = *it;
        if(mst[edge->a] == nullptr) mst[edge->a] = new list<int>;
        mst[edge->a]->push_back(edge->b);

        if(mst[edge->b] == nullptr) mst[edge->b] = new list<int>;
        mst[edge->b]->push_back(edge->a);
    }

    return;
}



// build min-weight perfect matching via D. Vinkemeier and S. Hougardy's approximation algo.
void build_matching(map<int, list<int>* > &mst, vector<vector<float>* > &dist, list<pair<int, int>* > &matching) {
    // find odd-degree vertices
    list<int> odd_list;
    for(auto it = mst.begin(); it != mst.end(); it++) {
        if(it->second->size() % 2 == 1) odd_list.push_back(it->first);
    }

    // by the handshaking lemma, there must now be an even number of odd vertices in the graph
    assert(odd_list.size() % 2 == 0);

    // build min-weight perfect matching
        // optimal is O(|V|^2 * |E|) [see Blossom V by V. Kolmogorov (2009)]
        // so we run a greedy version instead [see "A linear-time approx..." by D. Vinkemeier and S. Hougardy (2003)]
        // provides a 1/2 max-weight approximation in O(m log n) time

    priority_queue<Edge*, vector<Edge*>, DecrCompare> pq;
    for(int i : odd_list) {
        for(int j : odd_list) {
            if(i == j) continue;
            Edge* edge = new Edge(i, j, (*dist[i])[j]);
            pq.push(edge);
        }
    }

    set<int> matched_vertices;
    while(!pq.empty() && matched_vertices.size() < odd_list.size()) {
        Edge* edge = pq.top();
        pq.pop();
        if(matched_vertices.insert(edge->a).second) {
            if(matched_vertices.insert(edge->b).second) {
                // successful match
                pair<int, int>* ab = new pair<int, int>(edge->a, edge->b);
                pair<int, int>* ba = new pair<int, int>(edge->b, edge->a);
                matching.push_back(ab);
                matching.push_back(ba);
            } else {
                // a is unmatched but b is matched.
                matched_vertices.erase(edge->a);
            }
        }
    }

    assert(matched_vertices.size() == odd_list.size());

    return;
}



// build eulerian tour in graph via Hierholzer's algorithm
void build_eulerian_tour(map<int, list<int>* > &mst,
                         list<int> &eulerian_tour) {
    list<int> cur_path;
    cur_path.push_back(mst.begin()->first); // start with an element; doesn't really matter which.

    map<int, list<int>::iterator> adj_remaining;
    for(auto it = mst.begin(); it != mst.end(); it++) {
        adj_remaining[it->first] = it->second->begin();
    }

    while(cur_path.size() > 0) {
        int cur = cur_path.back();
        if(adj_remaining[cur] != mst[cur]->end()) {
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



void build_hamiltonian_cycle(list<int> &eulerian_tour, list<int> &hamiltonian_cycle) {
    map<int, bool> visited;
    for(uint64_t i = 0; i <= eulerian_tour.size(); i++) {
        visited[i] = false;
    }

    list<int>::iterator it = eulerian_tour.begin();
    hamiltonian_cycle.push_back(*it);
    visited[*it] = true;
    it++;

    while(it != eulerian_tour.end()) {
        if(!visited[*it]) {
            hamiltonian_cycle.push_back(*it);
            visited[*it] = true;
        }
        it++;
    }
}



// void construct_final_path(list<pair<float, float> > &output, int **prev, 
//                           list<int> hamiltonian_path, pair<float, float> starting_node,
//                           map<int, Point> point_map, map<Point, int> index_map) {

//     int start_idx = index_map[Point(starting_node.first, starting_node.second)];
//     std::cout << "start_idx: " << start_idx << std::endl;
//     list<int>::iterator it = std::find(hamiltonian_path.begin(), hamiltonian_path.end(), start_idx);
//     assert(it != hamiltonian_path.end());

//     output.push_back(pair<float, float>(starting_node.first, starting_node.second)); // deep copy

//     int u_idx, v_idx;
//     list<pair<float, float> > subpath;

//     std::cout << "2" << std::endl;

//     // go from our custom starting point to the end
//     while(true) {
//         u_idx = *it;
//         it++;
//         if(it == hamiltonian_path.end()) break;

//         v_idx = *it;
//         subpath.clear();
//         subpath.push_front(pair<float, float>(point_map[v_idx].x, point_map[v_idx].y));
//         while(u_idx != v_idx) {
//             v_idx = prev[u_idx][v_idx];
//             subpath.push_front(pair<float, float>(point_map[v_idx].x, point_map[v_idx].y));
//         }
//         output.splice(output.end(), subpath);
//     }

//     std::cout << "3" << std::endl;

//     // handle loop from end to start
//     v_idx = *hamiltonian_path.begin();
//     subpath.clear();
//     subpath.push_front(pair<float, float>(point_map[v_idx].x, point_map[v_idx].y));
//     while(u_idx != v_idx) {
//         v_idx = prev[u_idx][v_idx];
//         subpath.push_front(pair<float, float>(point_map[v_idx].x, point_map[v_idx].y));
//     }
//     output.splice(output.end(), subpath);

//     std::cout << "4" << std::endl;

//     // aaaand now go from start to custom starting point
//     list<int>::iterator stop = std::find(hamiltonian_path.begin(), hamiltonian_path.end(), start_idx);
//     it = hamiltonian_path.begin();
//     while(true) {
//         if(it == stop) break;
//         u_idx = *it;
//         it++;
//         v_idx = *it;
//         subpath.clear();
//         subpath.push_front(pair<float, float>(point_map[v_idx].x, point_map[v_idx].y));
//         while(u_idx != v_idx) {
//             v_idx = prev[u_idx][v_idx];
//             subpath.push_front(pair<float, float>(point_map[v_idx].x, point_map[v_idx].y));
//         }
//         output.splice(output.end(), subpath);
//     }

//     std::cout << "5" << std::endl;

// }



void simple_construct_path(list<pair<float, float>* > &output, vector<vector<int>* > prev, 
                          list<int> &hamiltonian_path, pair<float, float> starting_node,
                          map<int, Point> point_map, map<Point, int> index_map) {
    list<int>::iterator it = hamiltonian_path.begin();
    assert(it != hamiltonian_path.end());
    list<int>::iterator next = hamiltonian_path.begin();
    next++;

    while(next != hamiltonian_path.end()) {
        int u_idx = *it;
        int v_idx = *next;
        assert(u_idx != v_idx);

        list<pair<float, float>* > sublist;
        assert(v_idx != 0);
        assert(v_idx != -1);
        pair<float, float>* edge = new pair<float, float>(point_map[v_idx].x, point_map[v_idx].y);
        sublist.push_front(edge);

        v_idx = (*prev[u_idx])[v_idx];

        while(u_idx != v_idx) {
            assert(v_idx != 0);
            assert(v_idx != -1);
            pair<float, float>* edge_prime = new pair<float, float>(point_map[v_idx].x, point_map[v_idx].y);
            sublist.push_front(edge_prime);
            v_idx = (*prev[u_idx])[v_idx];
        }
        output.splice(output.end(), sublist);

        it++;
        next++;
    }

    
    for(list<pair<float, float>* >::iterator out_it = output.begin(); out_it != output.end(); out_it++) {
        if((*out_it)->first != starting_node.first || (*out_it)->second != starting_node.second) 
            continue;

        // put our start node at the front of the path
        output.splice(output.begin(), output, out_it, output.end());

        // add a path back from the last node to the start node
        int u_idx = index_map[Point(output.back()->first, output.back()->second)];
        int v_idx = index_map[Point(starting_node.first, starting_node.second)];
        list<pair<float, float>* > sublist;
        pair<float, float>* edge = new pair<float, float>(point_map[v_idx].x, point_map[v_idx].y);
        sublist.push_front(edge);
        v_idx = (*prev[u_idx])[v_idx];

        while(u_idx != v_idx) {
            assert(v_idx != 0);
            assert(v_idx != -1);
            pair<float, float>* edge_prime = new pair<float, float>(point_map[v_idx].x, point_map[v_idx].y);
            sublist.push_front(edge_prime);
            v_idx = (*prev[u_idx])[v_idx];
        }

        output.splice(output.end(), sublist);

        break;
    }

    // list<pair<float, float>* >::iterator start_point = std::find(output.begin(), output.end(), starting_node);
    // assert(start_point != output.end());
    // output.splice(output.begin(), output, start_point, output.end());
    // assert(output.front()->first == output.back()->first);
    // assert(output.front()->second == output.back()->second);
}



void coverage_planner::construct_global_coverage_path(map<pair<float, float>, list<pair<float, float> > > &edge_map,
                                    pair<float, float> starting_node, list<pair<float, float>* > &output) {
    
    std::cout << ">> starting global coverage path construction..." << std::endl;


    // 1. trim map down to a single connected component C.
    map<Point, list<Point>* > trimmed_map;
    trim_graph(edge_map, trimmed_map, starting_node);

    std::cout << ">> 1) trim completed..." << std::endl;


    // 2. assign vertex IDs. 
    map<Point, int> index_map;
    map<int, Point> point_map;
    uint64_t last_id = assign_vertex_ids(point_map, index_map, trimmed_map);

    std::cout << ">> 2) vertex ids assigned..." << std::endl;


    // 3. build all-pairs-shortest-paths solution via Floyd-Warshall.
    vector<vector<float>*> dist;
    for(uint64_t i = 0; i <= last_id; i++) {
        vector<float> *inner_dist = new vector<float>;
        for(uint64_t j = 0; j <= last_id; j++) inner_dist->push_back(std::numeric_limits<float>::infinity());
        dist.push_back(inner_dist);
    }
    vector<vector<int>*> prev;
    for(uint64_t i = 0; i <= last_id; i++) {
        vector<int> *inner_prev = new vector<int>;
        for(uint64_t j = 0; j <= last_id; j++) inner_prev->push_back(-1);
        prev.push_back(inner_prev);
    }
    floyd_warshall(trimmed_map, index_map, point_map, dist, prev, last_id);
    trimmed_map.clear();

    std::cout << ">> 3) all-pairs shortest paths constructed..." << std::endl;


    // let C' be the fully-connected version of C


    // 4. build a minimum spanning tree T in C'
    map<int, list<int>* > mst;
    build_mst(dist, last_id, mst);

    std::cout << ">> 4) minimum spanning tree constructed..." << std::endl;


    // 5. build a perfect matching M of odd-degree vertices in C'
    list<pair<int, int>* > matching;
    build_matching(mst, dist, matching);

    std::cout << ">> 5) perfect matching constructed..." << std::endl;


    // 6. combine T and M to make multigraph T'
    for(pair<int, int>* pair : matching) {
        list<int>* adj_list = mst[pair->first];
        if(std::find(adj_list->begin(), adj_list->end(), pair->second) == adj_list->end()) {
            adj_list->push_back(pair->second);
        }
    }
    matching.clear();

    std::cout << ">> 6) multigraph T' constructed..." << std::endl;


    // 6. identify a eulerian tour E in T'
    list<int> eulerian_tour;
    build_eulerian_tour(mst, eulerian_tour);
    mst.clear();

    std::cout << ">> 7) eulerian tour constructed..." << std::endl;


    // 7. reduce E to a hamiltonian cycle H
    list<int> hamiltonian_cycle;
    build_hamiltonian_cycle(eulerian_tour, hamiltonian_cycle);
    eulerian_tour.clear();

    std::cout << ">> 8) hamiltonian cycle constructed..." << std::endl;


    // 8. reconstruct paths via floyd-warshall prev list
    // construct_final_path(output, prev, hamiltonian_path, starting_node, point_map, index_map);
    simple_construct_path(output, prev, hamiltonian_cycle, starting_node, point_map, index_map);

    std::cout << ">> 9) final output path built..." << std::endl;


    return;
}
