// We model each emitter as a binary choice (horizontal or vertical).
// For each choice, we trace laser rays and record which empty cells it covers.
// The problem becomes a constraint satisfaction task: every cell must be covered
// by at least one chosen option, and each emitter picks exactly one valid option.
// We solve it via DFS with constraint propagation (forced choices).



#include <bits/stdc++.h>
using namespace std;

struct Option {
    bitset<2500> cover;
    bool valid = true;
};

int R, C;
vector<string> grid;
vector<pair<int,int>> emitters;
map<pair<int,int>, int> cell_id;
int empty_cnt;

vector<array<Option,2>> options;

int dr[4] = {0, -1, 0, 1}; // right, up, left, down
int dc[4] = {1, 0, -1, 0};

int reflect_slash(int d) {
    if (d == 0) return 1;
    if (d == 1) return 0;
    if (d == 2) return 3;
    return 2;
}

int reflect_backslash(int d) {
    if (d == 0) return 3;
    if (d == 3) return 0;
    if (d == 2) return 1;
    return 2;
}

void trace_ray(int sr, int sc, int dir, Option &opt) {
    int r = sr, c = sc, d = dir;
    while (true) {
        r += dr[d];
        c += dc[d];
        if (r < 0 || r >= R || c < 0 || c >= C) return;
        char ch = grid[r][c];
        if (ch == '#') return;
        if (ch == '-' || ch == '|') {
            opt.valid = false;
            return;
        }
        if (ch == '.') {
            opt.cover.set(cell_id[{r,c}]);
        } else if (ch == '/') {
            d = reflect_slash(d);
        } else if (ch == '\\') {
            d = reflect_backslash(d);
        }
    }
}

bool dfs(vector<int> &assign,
         vector<vector<pair<int,int>>> &cell_opts) {

    bool changed = true;
    while (changed) {
        changed = false;

        // emitter forced?
        for (int i = 0; i < (int)assign.size(); i++) {
            if (assign[i] != -1) continue;
            int cnt = 0, last = -1;
            for (int o = 0; o < 2; o++) {
                if (options[i][o].valid) {
                    cnt++; last = o;
                }
            }
            if (cnt == 0) return false;
            if (cnt == 1) {
                assign[i] = last;
                changed = true;
            }
        }

        // cell forced?
        for (int c = 0; c < empty_cnt; c++) {
            int cnt = 0;
            pair<int,int> last;
            for (auto &p : cell_opts[c]) {
                int e = p.first, o = p.second;
                if (assign[e] == o) {
                    cnt = 2;
                    break;
                }
                if (assign[e] == -1 && options[e][o].valid) {
                    cnt++;
                    last = p;
                }
            }
            if (cnt == 0) return false;
            if (cnt == 1) {
                assign[last.first] = last.second;
                changed = true;
            }
        }
    }

    int e = -1;
    for (int i = 0; i < (int)assign.size(); i++)
        if (assign[i] == -1) { e = i; break; }

    if (e == -1) return true;

    for (int o = 0; o < 2; o++) {
        if (!options[e][o].valid) continue;
        vector<int> na = assign;
        na[e] = o;
        if (dfs(na, cell_opts)) return true;
    }
    return false;
}

int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    int T;
    cin >> T;
    for (int tc = 1; tc <= T; tc++) {
        cin >> R >> C;
        grid.resize(R);
        for (int i = 0; i < R; i++) cin >> grid[i];

        emitters.clear();
        cell_id.clear();
        empty_cnt = 0;

        for (int i = 0; i < R; i++)
            for (int j = 0; j < C; j++) {
                if (grid[i][j] == '-' || grid[i][j] == '|')
                    emitters.push_back({i,j});
                if (grid[i][j] == '.')
                    cell_id[{i,j}] = empty_cnt++;
            }

        int N = emitters.size();
        options.assign(N, array<Option,2>());

        for (int i = 0; i < N; i++) {
            int r = emitters[i].first;
            int c = emitters[i].second;
            trace_ray(r, c, 0, options[i][0]);
            trace_ray(r, c, 2, options[i][0]);
            trace_ray(r, c, 1, options[i][1]);
            trace_ray(r, c, 3, options[i][1]);
        }

        bool ok = true;
        for (int i = 0; i < N; i++)
            if (!options[i][0].valid && !options[i][1].valid)
                ok = false;

        vector<vector<pair<int,int>>> cell_opts(empty_cnt);
        for (int i = 0; i < N; i++)
            for (int o = 0; o < 2; o++)
                if (options[i][o].valid)
                    for (int c = 0; c < empty_cnt; c++)
                        if (options[i][o].cover.test(c))
                            cell_opts[c].push_back({i,o});

        for (int c = 0; c < empty_cnt; c++)
            if (cell_opts[c].empty())
                ok = false;

        bool possible = false;
        if (ok) {
            vector<int> assign(N, -1);
            possible = dfs(assign, cell_opts);
        }

        cout << "Case #" << tc << ": "
             << (possible ? "POSSIBLE" : "IMPOSSIBLE") << "\n";
    }
}
