#include <bits/stdc++.h>
using namespace std;


struct AStar {
    vector<vector<bool>> worldMap;
    set<int> st;
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
    auto &worldMap = aStar.worldMap;
    string buf;
    int height, width;
    cin >> buf >> height >> buf >> width;
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
    return 0;
}