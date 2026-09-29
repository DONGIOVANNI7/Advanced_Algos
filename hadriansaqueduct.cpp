// max-flow with Dinic — BFS builds levels, DFS pushes flow, repeat until stuck
// O(E sqrt V) in practice
// capacities up to 1e9 → we use long long
// nodes 1..n → shift to 0..n-1 inside
//Another solution could be Edmonds-Karp (BFS-based Ford-Fulkerson) - O(VE²)

#include <bits/stdc++.h>
using namespace std;

struct Dinic {
    struct Edge {
        int to;
        long long cap;
        int rev;
    };
    
    int N;
    vector<vector<Edge>> G;
    vector<int> level, prog;
    
    Dinic(int n) : N(n), G(n), level(n), prog(n) {}
    
    void addEdge(int fr, int to, long long cap) {
        Edge a{to, cap, (int)G[to].size()};
        Edge b{fr, 0, (int)G[fr].size()};
        G[fr].push_back(a);
        G[to].push_back(b);
    }
    
    bool bfs(int s, int t) {
        fill(level.begin(), level.end(), -1);
        queue<int> q;
        level[s] = 0;
        q.push(s);
        while (!q.empty()) {
            int v = q.front(); q.pop();
            for (const Edge& e : G[v]) {
                if (e.cap > 0 && level[e.to] < 0) {
                    level[e.to] = level[v] + 1;
                    q.push(e.to);
                }
            }
        }
        return level[t] >= 0;
    }
    
    long long dfs(int v, int t, long long f) {
        if (v == t) return f;
        for (int& i = prog[v]; i < (int)G[v].size(); ++i) {
            Edge& e = G[v][i];
            if (e.cap > 0 && level[e.to] == level[v] + 1) {
                long long ret = dfs(e.to, t, min(f, e.cap));
                if (ret > 0) {
                    e.cap -= ret;
                    G[e.to][e.rev].cap += ret;
                    return ret;
                }
            }
        }
        return 0;
    }
    
    long long maxFlow(int s, int t) {
        long long flow = 0;
        const long long INF = LLONG_MAX;
        while (bfs(s, t)) {
            fill(prog.begin(), prog.end(), 0);
            long long f;
            while ((f = dfs(s, t, INF)) > 0) {
                flow += f;
            }
        }
        return flow;
    }
};

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);
    
    int n, m;
    if (!(cin >> n >> m)) return 0;
    
    Dinic dinic(n);
    for (int i = 0; i < m; ++i) {
        int a, b;
        long long c;
        cin >> a >> b >> c;
        dinic.addEdge(a - 1, b - 1, c); // Convert to 0-indexed
    }
    
    long long result = dinic.maxFlow(0, n - 1);
    cout << result << "\n";
    
    return 0;
}
