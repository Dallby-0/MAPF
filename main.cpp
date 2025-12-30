#include <bits/stdc++.h>
using namespace std;

//const double sq2 = 1.41421356237309504880;

namespace M{
    int height, width;
    vector<vector<bool>> worldMap;
}

namespace D {
    // //URDL UR RD DL LU
    // int dx[9] = {-1,0,1,0,-1,1,1,-1,0};
    // int dy[9] = {0,1,0,-1,1,1,-1,-1,0};
    //被ai骗了 本来以为找的最优解网站支持八向 特意写的8向
    int dx[5] = {-1,0,1,0,0};
    int dy[5] = {0,1,0,-1,0};
}

namespace Constraint {
    enum {
        up,right,down,left,UR,RD,DL,LU,pos
    };//pos指某时不能存在于该点,而方向则表示某时处于该点时不能选择某方向

    int x,y,t,dir;
    int encode(int x, int y, int t, int dir){
        int res = t * M::height;
        res = (res + x) * M::width;
        res = (res + y) * 5 + dir;
        return res;
    }
    // Constraint (int x, int y, int t, int dir):x(x),y(y),t(t),dir(dir) {}
    // Constraint (int code){ //decode 似乎没啥用
    //
    // }
};

struct AStarNode {
    int x, y, t;
    int predictedCost;
    bool operator<(const AStarNode &rhs) const {
        return predictedCost > rhs.predictedCost;
    }
    int encode() { //注意x对应height 范围为0 ~ height-1
        return t * M::height * M::width + x * M::width + y;
    }
    AStarNode(int x, int y, int t, int predictedCost): x(x), y(y), t(t), predictedCost(predictedCost){}
    AStarNode(int code) { //注意x对应height 范围为0 ~ height-1
        y = code % M::width;
        code /= M::width;
        x = code % M::height;
        t = code / M::height;
        predictedCost = 0;
    }
    AStarNode() {
        x = y = t = predictedCost = 0;
    }
};


struct Point {
    int x, y;
    Point(const int x, const int y):x(x),y(y){}
    Point(){x=0, y=0;}
};

struct AStar {
    //set<int> blockedCodes;
    int minCost(int sx, int sy, int ex, int ey) {
        //被ai骗了 本来以为找的最优解网站支持八向 特意写的8向
        // int dx = abs(ex - sx);
        // int dy = abs(ey - sy);
        // int mi = min(dx, dy);
        // int mx = max(dx, dy);
        // double res = sq2 * mi + mx - mi;
        //return res;
        return abs(ex - sx) + abs(ey - sy);
    }
    bool findPath (int sx, int sy, int ex, int ey, const set<int> &blockedCodes, int lastConstraintTime, vector<Point> &path) {//考虑
        // auto cmp = [=](const AStarNode &lhs, const AStarNode &rhs) {
        //     return lhs.t + minCost(lhs.x, lhs.y, ex, ey) > rhs.t + minCost(rhs.x, rhs.y, ex, ey);
        // };
        //priority_queue<AStarNode,vector<AStarNode>, decltype(cmp)> pq(cmp); //这种情况不太适合用operator 结果被迫学习c++特性
        priority_queue<AStarNode> pq;
        auto initialNode = AStarNode(sx, sy, 0, minCost(sx, sy, ex, ey));
        pq.push(initialNode);
        map<int, int> preMap;
        set<int> visited; //其实可以用preMap代替visited，随便了，可读性>常数
        set<pair<int,int>> visitedAfterConstraints; //所有约束结束之后，等效于普通A*，所以记录空间访问
        bool ok = false;
        int finalNodeCode;
        while (!pq.empty()) {
            AStarNode node = pq.top();
            pq.pop();
            if (node.x == ex && node.y == ey) {
                ok = true;
                finalNodeCode = node.encode();
                break;
            }
            for (int k = 0; k < 5; k++) {
                if (k < 4) {
                    //规定在当前点不能进行朝向为k的动作（边冲突）
                    if (int codeAction = Constraint::encode(node.x, node.y, node.t, k); blockedCodes.contains(codeAction)){
                    continue;
                    }
                    //点了下编译器代码优化成这一行了  这是c++xx？
                }
                int nx = node.x + D::dx[k];
                int ny = node.y + D::dy[k];
                if (M::worldMap[nx][ny]) {
                    continue;
                }
                int nt = node.t + 1;
                if (nt > lastConstraintTime) {

                }
                if (int codeNext = Constraint::encode(nx, ny, nt, 4); blockedCodes.contains(codeNext))
                    continue;
                AStarNode newNode(nx, ny, nt, nt + minCost(nx, ny, ex, ey));
                int newNodeCode = newNode.encode();
                if (visited.contains(newNodeCode)) {
                    continue;
                }
                visited.insert(newNodeCode);
                pq.push(newNode);
                preMap[newNodeCode] = node.encode();
                //int cost = node.cost + (k >= 4 ? sq2 : 1);
            }
        }
        if (ok) {
            path.clear();
            int currentNodeCode = finalNodeCode;
            while (preMap.contains(currentNodeCode)) {
                //path.emplace_back(currentNodeCode);
                AStarNode node(currentNodeCode);
                path.emplace_back(node.x, node.y); //我靠 原来emplace_back这么好用
                currentNodeCode = preMap[currentNodeCode];
            }
            //不需要包含七点所以不用下面两行
            //AStarNode node(currentNodeCode);
            //path.emplace_back(sx, sy);
            ranges::reverse(path); //ide的提示总会给出一些高级的东西。。
            return true;
        }
        return false;
    }
}aStar;

struct CBSPlanner {

}planner;

void init() {
    const string map = "Berlin_1_256";
    const bool useEven = true;
    const int id = 1;
    const string scen =  useEven ? "scen-even" : "scen-random";
    const string fileName = map + "-" + (useEven ? "even" : "random") + "-" + to_string(id);
    const string scenPath = "../maps/" + map + "/" + scen + "/" + fileName + ".scen";
    const string mapPath = "../maps/" + map + "/" + map + ".map";
    cout << "mapPath:" << mapPath << endl;
    cout << "scenPath:" << scenPath << endl;
    //read map
    freopen(mapPath.c_str(), "r", stdin);
    string line;
    getline(cin, line);
    cout << line <<endl;
    auto &worldMap = M::worldMap;
    string buf;
    int height, width;
    cin >> buf >> height >> buf >> width;
    M::height = height;
    M::width = width;
    cout <<"height/width: " << height << " " << width << endl;
    for (int i = 0; i < height; i++) {
        getline(cin, line);
        worldMap.emplace_back();
        for (char ch : line) {
            if (ch == '.') worldMap[i].push_back(false);
            else worldMap[i].push_back(true);
        }
    }
}

int main() {
    init();
    vector<Point> path;
    set<int> blockedCodes;
    aStar.findPath(92, 220, 65, 194, blockedCodes, 1e9, path);
    cout << path.size() << endl;
    for (auto p : path) {
        cout << p.x << " " << p.y << endl;
    }
    return 0;
}