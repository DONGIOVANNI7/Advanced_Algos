//I give a brief description of the algorithm:
//1. Y-coordinate compression: split the grid at every building y0 and y1+1 so we get variable-height horizontal strips (height = h).
//2. Corner/strip graph: build nodes for grid corners (x = 0..W) at each strip boundary (s = 0..num_strips).
//3. Edge costs:
//   - Horizontal: cost 1 except cost 0 when an adjacent cell (above or below the edge) is blocked (wall-hugging).
//   - Vertical: crossing a strip s costs its height h (or 0 when adjacent cells block the crossing).
//   - Paying cost h allows lateral jumps up to distance h (models cutting across that strip).
//4. Hub optimization: when a strip's height h ≥ W i use a single "hub" node for that strip to connect to every corner of the next row efficiently
//5. Shortest-path solver: run monotone Dijkstra on this integer-weight graph using a radix heap (keys are nondecreasing) for speed.
//6. Interpretation: the computed minimum cut cost on the compressed geometry equals the original maximum flow (vertex-capacity) of the W×H grid.
//- All costs use strip height h (not global H).
//- The compression + jump model is exact (no loss of optimality) and far smaller than the full W×H graph.
//My derived Time Complexity: O(T x W x (B + min(H, B x W)))
// Space Complexity : O(B x W) 

//#pragma GCC optimize("Ofast,unroll-loops")
//#pragma GCC target("avx2,bmi,bmi2,lzcnt,popcnt")
#include <bits/stdc++.h>
using namespace std;
using ll = long long;
using ull = unsigned long long;
const ll INF = (ll)1e18;

struct Building { int x0,y0,x1,y1; };

// Radix heap for monotone non-decreasing 64-bit keys
template<typename Key, typename Value>
class RadixHeap {
    static int bsr(ull x){
        return x? 63 - __builtin_clzll(x) : -1;
    }
    vector<pair<Key,Value>> buf; // temporary
    array<vector<pair<Key,Value>>, 65> buckets;
    Key last;
    size_t sz;
public:
    RadixHeap(): last(0), sz(0) {}
    inline bool empty() const { return sz==0; }
    inline size_t size() const { return sz; }
    inline void push(Key key, Value val){
        // key must be >= last
        ull diff = (ull)(key ^ last);
        int idx = bsr(diff) + 1; // if diff==0 -> -1+1 = 0 -> bucket 0
        buckets[idx].emplace_back(key, val);
        ++sz;
    }
    inline pair<Key,Value> pop(){
        if(buckets[0].empty()){
            int i = 1;
            while(i < 65 && buckets[i].empty()) ++i;
            // find minimal key in bucket i
            Key mn = numeric_limits<Key>::max();
            for(auto &p : buckets[i]) if(p.first < mn) mn = p.first;
            last = mn;
            // redistribute
            for(auto &p : buckets[i]){
                ull diff = (ull)(p.first ^ last);
                int idx = bsr(diff) + 1;
                buckets[idx].push_back(p);
            }
            buckets[i].clear();
        }
        auto p = buckets[0].back();
        buckets[0].pop_back();
        --sz;
        return p;
    }
};

int main(){
    ios::sync_with_stdio(false);
    cin.tie(nullptr);
    int T;
    if(!(cin>>T)) return 0;
    for(int tc=1; tc<=T; ++tc){
        int W, B;
        ll H;
        cin >> W >> H >> B;
        vector<Building> buildings;
        buildings.reserve(B);
        for(int i=0;i<B;++i){
            Building b; cin >> b.x0 >> b.y0 >> b.x1 >> b.y1;
            buildings.push_back(b);
        }

        // y-coordinates
        vector<ll> ys;
        ys.reserve(2 + 2*B);
        ys.push_back(0);
        ys.push_back(H);
        for(const auto &b : buildings){
            ys.push_back(b.y0);
            ys.push_back((ll)b.y1 + 1);
        }
        sort(ys.begin(), ys.end());
        ys.erase(unique(ys.begin(), ys.end()), ys.end());
        int num_strips = (int)ys.size() - 1;
        if(num_strips <= 0){
            cout << "Case #" << tc << ": 0\n";
            continue;
        }

        vector<ll> strip_h(num_strips);
        for(int s=0;s<num_strips;++s) strip_h[s] = ys[s+1] - ys[s];

        // blocked grid
        vector<unsigned char> is_blocked((size_t)num_strips * (size_t)W, 0);
        vector<int> row_base(num_strips);
        for(int s=0;s<num_strips;++s) row_base[s] = s * W;

        for(const auto &b : buildings){
            int s_start = (int)(lower_bound(ys.begin(), ys.end(), (ll)b.y0) - ys.begin());
            int s_end = (int)(lower_bound(ys.begin(), ys.end(), (ll)b.y1 + 1) - ys.begin()) - 1;
            if(s_start < 0) s_start = 0;
            if(s_end >= num_strips) s_end = num_strips - 1;
            if(s_start > s_end) continue;
            int x0 = max(0, b.x0);
            int x1 = min(W-1, b.x1);
            if(x0 > x1) continue;
            for(int s = s_start; s <= s_end; ++s){
                int base = row_base[s];
                for(int x = x0; x <= x1; ++x) is_blocked[base + x] = 1;
            }
        }

        const int ncx = W + 1;
        const int nr = num_strips + 1;
        const int num_corners = ncx * nr;
        const int hub_start_idx = num_corners;
        const int total_nodes = num_corners + num_strips;

        vector<int> corner_base(ncx);
        for(int cx=0; cx<ncx; ++cx) corner_base[cx] = cx * nr;

        // distances
        vector<ll> dist(total_nodes, (ll)INF);
        RadixHeap<ll,int> pq;
        // init left bank x=0
        for(int cs=0; cs<nr; ++cs){
            int idx = corner_base[0] + cs;
            dist[idx] = 0;
            pq.push(0, idx);
        }

        ll ans = INF;

        while(!pq.empty()){
            auto p = pq.pop();
            ll d = p.first;
            int u = p.second;
            if(d != dist[u]) continue;
            if(d >= ans) break; // early exit, no better path possible

            if(u >= hub_start_idx){
                int s = u - hub_start_idx;
                int target_row = s + 1;
                int base_row = target_row;
                // push all corners in target_row
                for(int cx=0; cx<ncx; ++cx){
                    int v = corner_base[cx] + base_row;
                    if(d < dist[v]){
                        dist[v] = d;
                        pq.push(d, v);
                    }
                }
                continue;
            }

            int cx = u / nr;
            int cs = u % nr;
            if(cx == W){
                if(d < ans) ans = d;
                continue;
            }

            // RIGHT
            {
                int v = corner_base[cx+1] + cs;
                long long w = 1;
                if(cs < num_strips){
                    if(is_blocked[row_base[cs] + cx]) w = 0;
                }
                if(cs > 0){
                    if(is_blocked[row_base[cs-1] + cx]) w = 0;
                }
                ll nd = d + w;
                if(nd < dist[v]) { dist[v] = nd; pq.push(nd, v); }
            }

            // LEFT
            if(cx > 0){
                int v = corner_base[cx-1] + cs;
                long long w = 1;
                if(cs < num_strips){
                    if(is_blocked[row_base[cs] + (cx-1)]) w = 0;
                }
                if(cs > 0){
                    if(is_blocked[row_base[cs-1] + (cx-1)]) w = 0;
                }
                ll nd = d + w;
                if(nd < dist[v]) { dist[v] = nd; pq.push(nd, v); }
            }

            // UP
            if(cs < num_strips){
                ll h = strip_h[cs];
                int rb = row_base[cs];
                bool free_vert = false;
                if(cx < W && is_blocked[rb + cx]) free_vert = true;
                if(cx > 0 && is_blocked[rb + (cx-1)]) free_vert = true;
                if(free_vert){
                    int v = corner_base[cx] + (cs+1);
                    if(d < dist[v]) { dist[v] = d; pq.push(d, v); }
                } else {
                    if(h >= W){
                        int hub_idx = hub_start_idx + cs;
                        ll nd = d + h;
                        if(nd < dist[hub_idx]) { dist[hub_idx] = nd; pq.push(nd, hub_idx); }
                    } else {
                        int start_k = cx - (int)h; if(start_k < 0) start_k = 0;
                        int end_k = cx + (int)h; if(end_k > W) end_k = W;
                        int base_row = cs + 1;
                        ll nd = d + h;
                        for(int nx = start_k; nx <= end_k; ++nx){
                            int v = corner_base[nx] + base_row;
                            if(nd < dist[v]) { dist[v] = nd; pq.push(nd, v); }
                        }
                    }
                }
            }

            // DOWN
            if(cs > 0){
                ll h = strip_h[cs-1];
                int rb = row_base[cs-1];
                bool free_vert = false;
                if(cx < W && is_blocked[rb + cx]) free_vert = true;
                if(cx > 0 && is_blocked[rb + (cx-1)]) free_vert = true;
                if(free_vert){
                    int v = corner_base[cx] + (cs-1);
                    if(d < dist[v]) { dist[v] = d; pq.push(d, v); }
                } else {
                    if(h >= W){
                        int v = corner_base[cx] + (cs-1);
                        ll nd = d + h;
                        if(nd < dist[v]) { dist[v] = nd; pq.push(nd, v); }
                    } else {
                        int start_k = cx - (int)h; if(start_k < 0) start_k = 0;
                        int end_k = cx + (int)h; if(end_k > W) end_k = W;
                        int base_row = cs - 1;
                        ll nd = d + h;
                        for(int nx = start_k; nx <= end_k; ++nx){
                            int v = corner_base[nx] + base_row;
                            if(nd < dist[v]) { dist[v] = nd; pq.push(nd, v); }
                        }
                    }
                }
            }
        } // while

        if(ans == INF) ans = -1;
        cout << "Case #" << tc << ": " << ans << "\n";
    }
    return 0;
}
