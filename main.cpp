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
    enum{U,R,D,L,O};
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
    Point(){x=-1, y=-1;}
    bool operator < (const Point &rhs) const {
        if (x == rhs.x) {
            return y < rhs.y;
        }
        return x < rhs.x;
    }
    bool operator == (const Point &rhs) const {
        return x == rhs.x && y == rhs.y;
    }
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
        //lastConstraintTime += 30;
        cout << "astar " << sx <<" " << sy <<" " << ex <<" " << ey << " " << blockedCodes.size() << endl;
        priority_queue<AStarNode> pq;
        auto initialNode = AStarNode(sx, sy, 0, minCost(sx, sy, ex, ey));
        pq.push(initialNode);
        map<int, int> preMap;
        set<int> visited; //其实可以用preMap代替visited，随便了，可读性>常数
        set<pair<int,int>> visitedAfterConstraints; //所有约束结束之后，等效于普通A*，所以记录空间访问
        bool ok = false;
        int finalNodeCode;
        //int cnt = 0;
        while (!pq.empty()) {
            //cout << "aspq\n";
            AStarNode node = pq.top();
            //cnt ++;
            //if (cnt > 60000) {
            //}

            pq.pop();
            if (node.t > lastConstraintTime) {
                if (visitedAfterConstraints.count({node.x, node.y})) {
                    //cout <<" pass!" << endl;
                    continue;
                }
                visitedAfterConstraints.insert({node.x, node.y});
            }
            //if (cnt > 36000) {
                //cout << node.x << " " << node.y << " " << node.t << " " << node.predictedCost << " " << lastConstraintTime<<endl;
            //}
            //cout << node.x << " " << node.y << " " << node.t << " " << node.predictedCost << " " << lastConstraintTime<<endl;
            if (node.x == ex && node.y == ey) {
                ok = true;
                finalNodeCode = node.encode();
                break;
            }
            for (int k = 0; k < 5; k++) {
                if (k < 4) {
                    //规定在当前点不能进行朝向为k的动作（边冲突）
                    if (int codeAction = Constraint::encode(node.x, node.y, node.t, k); blockedCodes.contains(codeAction)){
                        cout << "ASTAEDGEC: " << node.x << " " << node.y << " " << node.t << k <<endl;
                        continue;
                    }
                    //点了下编译器代码优化成这一行了  这是c++xx？
                }
                int nx = node.x + D::dx[k];
                int ny = node.y + D::dy[k];
                if (nx < 0 || nx >= M::height || ny < 0 || ny >= M::width) {
                    continue;
                }
                if (M::worldMap[nx][ny]) {
                    continue;
                }
                int nt = node.t + 1;
                if (int codeNext = Constraint::encode(nx, ny, nt, 4); blockedCodes.contains(codeNext)) {
                    cout << "ASTARPOSC: " << nx << " " << ny << " " << nt <<endl;
                    continue;
                }
                AStarNode newNode(nx, ny, nt, nt + minCost(nx, ny, ex, ey));
                int newNodeCode = newNode.encode();
                if (nt > lastConstraintTime) {
                    if (visitedAfterConstraints.contains({newNode.x, newNode.y})) {
                        continue;
                    }
                }
                if (visited.contains(newNodeCode)) {
                    continue;
                }
                visited.insert(newNodeCode);
                pq.push(newNode);
                preMap[newNodeCode] = node.encode();
                //int cost = node.cost + (k >= 4 ? sq2 : 1);
            }
        }
        cout << "astar done " << ok <<endl;
        if (ok) {
            path.clear();
            int currentNodeCode = finalNodeCode;
            while (preMap.contains(currentNodeCode)) {
                //path.emplace_back(currentNodeCode);
                AStarNode node(currentNodeCode);
                path.emplace_back(node.x, node.y); //我靠 原来emplace_back这么好用
                currentNodeCode = preMap[currentNodeCode];
            }
            //不需要包含起点所以不用下面两行，但是我决定包含起点
            //AStarNode node(currentNodeCode);
            path.emplace_back(sx, sy);
            ranges::reverse(path); //ide的提示总会给出一些高级的东西。。
            return true;
        }
        return false;
    }
}aStar;

struct Conflict {
    //bool isEdgeConflict = false; //default Position conflict
    int idA, idB;
    Point posA, posB; //if posA == posB then is position conflict 另外时间和位置均为发生冲突之后
    int time;
    Conflict(int idA, int idB, Point posA, Point posB, int time):idA(idA),idB(idB),posA(posA),posB(posB),time(time){}
    Conflict(){}
};

struct CBSNode {//目前lastConstraintTime要手动设置
    bool calculated = false;
    bool ok; //是否可以在当前冲突下找到路径（不限制无冲突）
    bool hasConflict;
    vector<set<int>> blockedCodesList;
    int presumedMinCost, lastConstraintTime;
    Conflict firstConflict;
    bool operator < (const CBSNode &rhs) const { //for pq
        return presumedMinCost > rhs.presumedMinCost;
    }
};

struct Robot {
    int sx, sy, ex, ey;
};

struct CBSPlanner {
    vector<Robot> robots;
    int robotCount;
    void calculateNode(CBSNode &node) {
        cout << "cal\n";
        vector<vector<Point>> paths;
        paths.resize(robotCount);
        node.hasConflict = false;
        node.ok = true;
        for (int i = 0; i < robotCount; i++) {
            auto &robot = robots[i];
            bool havePath = aStar.findPath(robot.sx, robot.sy, robot.ex, robot.ey, node.blockedCodesList[i], node.lastConstraintTime, paths[i]);
            if (!havePath) {
                node.ok = false;
            }
        }
        if (node.ok) {//找新的冲突，低效实现
            cout << "cal1\n";
            int mxLen = 0;
            for (auto &v : paths) {
                mxLen = max(mxLen, (int)v.size());
            }
            //bool ok = true;


            for (int i = 0; i < mxLen; i++) {
                map<Point, int> positionToId;
                int id = 0;
                for (auto &v : paths) {
                    if (i < v.size()) {
                        if (positionToId.contains(v[i])) {
                            //位置冲突
                            cout << "found posconf:" << v[i].x <<" " << v[i].y <<endl;
                            node.hasConflict = true;
                            node.firstConflict = Conflict(positionToId[v[i]], id, v[i], v[i], i);
                            break;
                        }
                        positionToId[v[i]] = id;
                    }
                    id++;
                }
                //cout <<"i size: " << i << " " << positionToId.size() << endl;
                if (i > 0){
                    //先找目前是否有节点vb走到了上一轮<其他节点va>所在的位置，若有，则判断<其他节点va>本轮的位置是否与目前节点vb上一轮位置相同，很绕，所以写了注释
                    int idA = 0;
                    for (auto &va : paths) {
                        if (i < va.size()) {
                            if (positionToId.contains(va[i-1])) {
                                int idB = positionToId[va[i-1]];
                                auto &vb = paths[idB];
                                if (idA != idB && vb[i-1] == va[i]) {//考虑到idB在本轮有映射，所以不用考虑越界
                                    //边冲突
                                    cout << "found edgeconf\n";
                                    node.hasConflict = true;
                                    node.firstConflict = Conflict(idA, idB, va[i], vb[i], i);
                                    break;
                                }
                            }
                        }
                        idA++;
                    }
                }
                if (node.hasConflict) { //找到一个冲突即可
                    cout << "found conf\n";
                    break;
                }
            }
        }
        if (node.ok) {
            node.presumedMinCost = 0;
            for (auto &v : paths) { //暂时（由于只是作业，所以估计是永远）用不考虑新冲突的代价作为排序用代价
                node.presumedMinCost += v.size() - 1;
            }
            // for (auto & s : node.blockedCodesList) {
            //     node.presumedMinCost += s.size();
            // }
            // node.presumedMinCost += node.hasConflict;
        }
        node.calculated = true;
        cout << "cal done\n";
    }

    void solve() {
        CBSNode initialNode;
        initialNode.lastConstraintTime = 0;
        initialNode.blockedCodesList.resize(robotCount);
        calculateNode(initialNode);
        priority_queue<CBSNode> pq;
        pq.push(initialNode);
        int currentMinCost = INT_MAX;
        int dreamedMinCost = initialNode.presumedMinCost;
        while (!pq.empty()) {
            //cout << "hi";
            CBSNode node = pq.top();
            pq.pop();
            int totalCCount = 0;
            for (auto s : node.blockedCodesList) totalCCount += s.size();
            cout << "totalCCount: " << totalCCount << endl;
            if (node.presumedMinCost >= currentMinCost) {
                break;
            }
            if (!node.hasConflict) {
                currentMinCost = node.presumedMinCost;
                cout << "solved! cost: " << currentMinCost << " " << dreamedMinCost << endl;
            }
            else { //拆成两份塞进去计算！！
                cout << "conflict\n";
                CBSNode lch, rch;
                lch.blockedCodesList = rch.blockedCodesList = node.blockedCodesList;
                Conflict conflict = node.firstConflict;
                int cTime = conflict.time;
                int constraintCodeA, constraintCodeB;
                if (conflict.posA == conflict.posB) {
                    Point pos = conflict.posA;
                    int constraintCode = Constraint::encode(pos.x,pos.y,cTime,4);
                    constraintCodeA = constraintCodeB = constraintCode;
                }
                else {
                    //考虑到t是冲突发生后的时间，所以应该在t-1加入限制
                    cTime--;
                    int dirA, dirB;
                    if (conflict.posA.x != conflict.posB.x) {
                        //再次提醒，x是纵轴，因为x在前，i在前！
                        //另外这是冲突移动之后的坐标
                        //如果A的x较小，A最终在上，那意味着A向上了，所以禁止A向上
                        dirA = D::U;
                        dirB = D::D;
                        if (conflict.posA.x > conflict.posB.x) {
                            swap(dirA, dirB);
                        }
                    }
                    else {
                        dirA = D::L;
                        dirB = D::R;
                        if (conflict.posA.y > conflict.posB.y) {
                            swap(dirA, dirB);
                        }
                    }
                    Point pposA = conflict.posB, pposB = conflict.posA;
                    constraintCodeA = Constraint::encode(pposA.x, pposA.y, cTime, dirA);
                    constraintCodeB = Constraint::encode(pposB.x , pposB.y, cTime, dirB);
                }
                if (lch.blockedCodesList[conflict.idA].contains(constraintCodeA)) {
                    cout << "bug 重复禁止动作A\n";
                }
                if (rch.blockedCodesList[conflict.idB].contains(constraintCodeB)) {
                    cout << "bug 重复禁止动作B\n";
                }
                lch.blockedCodesList[conflict.idA].insert(constraintCodeA);
                rch.blockedCodesList[conflict.idB].insert(constraintCodeB);
                int lastConstraintTime = max(node.lastConstraintTime, cTime);
                lch.lastConstraintTime = rch.lastConstraintTime = lastConstraintTime;
                calculateNode(lch);
                calculateNode(rch);
                pq.push(lch);
                pq.push(rch);
            }
        }
    }
}planner;

void init() {
    //const string map = "Berlin_1_256";
    const string map = "maze-32-32-2";
    const bool useEven = true;
    const int id = 1;
    const string scen =  useEven ? "scen-even" : "scen-random";
    const string fileName = map + "-" + (useEven ? "even" : "random") + "-" + to_string(id);
    const string scenPath = "../maps/" + map + "/" + scen + "/" + fileName + ".scen";
    const string mapPath = "../maps/" + map + "/" + map + ".map";
    cout << "mapPath:" << mapPath << endl;
    cout << "scenPath:" << scenPath << endl;
    //read map
    auto fp = freopen(mapPath.c_str(), "r", stdin);
    string line;
    getline(cin, line);
    cout << line <<endl;
    auto &worldMap = M::worldMap;
    string buf;
    int height, width;
    cin >> buf >> height >> buf >> width;
    M::height = height;
    M::width = width;
     //漏了这两行 调试过于折磨
    cin >> line;
    cout << line <<endl;
    cout <<"height/width: " << height << " " << width << endl;
    for (int i = 0; i < height; i++) {
        cin >> line;
        worldMap.emplace_back();
        for (char ch : line) {
            if (ch == '.') worldMap[i].push_back(false);
            else worldMap[i].push_back(true);
        }
    }
    cout << "real map size " << worldMap.size() << " " << worldMap[0].size() << endl;
    clearerr(stdin); //我tm被这个问题坑死了
    cin.clear(); //tmd
    freopen(scenPath.c_str(), "r", stdin);

    line = "123";
    getline(cin, line);
    cout << line << endl;
    int cnt = 0;
    planner.robotCount = 0;
    while (getline(cin, line)) {
        if (++cnt > 8) break;
        stringstream ss(line);
        int bucket;
        string mapName;
        int h,w;
        Robot robot;
        ss >> bucket >> mapName >> h >> w >> robot.sy >> robot.sx >> robot.ey >> robot.ex;
        planner.robots.push_back(robot);
        planner.robotCount++;
    }
    cout << "init done\n";
}

int main() {
    init();
    vector<Point> path;
    set<int> blockedCodes;
    //aStar.findPath(92, 220, 65, 194, blockedCodes, 1e9, path);
    planner.solve();
    return 0;
}