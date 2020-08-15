#include <opencv2/opencv.hpp>
#include <cstdlib>
#include <algorithm>
#include <set>
#include <iostream>
#include <cstdlib>

using namespace cv;
using namespace std;


struct vertex{
    
    int up,down,right,left;
    int x,y;
    int p;
    bool visited = false;
    vector<vertex> adj;
};

struct graph{
    Mat gray_graph;
    vector<vector<int>> hor; // horizonal weight
    vector<vector<int>> ver; // vertical weight
    
};

vector<vertex> vertex_set;   // use set to save the vertexes
vector<vector<vertex>> all_adj;  // used to save the adj_relationship of the graph
vector<vertex> s,t; // s:foreground, t:background
vector<vertex> res_Graph;
int col;
int row;


void build_graph(Mat image);
void Ford_Fulkerson(vertex super_s,vertex super_t,Mat image);
bool bfs(vector<vertex> res_Graph, vertex super_s,vertex super_t,vector<vertex> &path, vector<int> &path2);
int get_min_cap(vector<vertex> res_Graph,vector<vertex> &path);
void update_res_graph(int &bottleNeck,vector<vertex> &path,vector<vertex> res_Graph);
void min_cut(vector<vertex> s,vector<vertex> t,vertex super_s,vertex super_t);


int main( int argc, char** argv )
{
    
    // Load the input image
    // the image should be a 3 channel image by default but we will double check that in teh seam_carving
    Mat in_image,out_image;
    in_image = imread("/Users/lekangdu/Desktop/input5.png"/*, CV_LOAD_IMAGE_COLOR*/);
    
    
    if(!in_image.data)
    {
        cout<<"Could not load input the image!"<<endl;
        return -1;
    }
    
    if(in_image.channels()!=3){
        cout<<"Image does not have 3 channels!"<<in_image.depth()<<endl;
        return -1;
    }
    
    ifstream f("/Users/lekangdu/Desktop/config5.txt");
    if(!f){
        cout<<"Could not open file."<<endl;
        return -1;
    }
    // read the initial pixel
    vertex super_s,super_t; // super_s: virtual source, super_t: virtual
    
    super_s.up = 100000;  // suppose all s are connected with super_s from up
    super_s.x = -1;  // suppose super_s (INT_MAX,INT_MAX)
    super_s.y = -1;
    
    super_t.down = 100000; // suppose all t are connected with super_t from down
    super_t.x = -2;  // suppose super_t (INT_MIN,INT_MIN)
    super_t.y = -2;
    
    // covert rgb image to gray image
    cvtColor(in_image,out_image,COLOR_RGB2GRAY);
    col = out_image.cols;
    row = out_image.rows;
    graph out_graph;
    out_graph.gray_graph = out_image;
    
    // use gray graph to save the vertex into vertex set
    // include the weights, xy, and adj;
    build_graph(out_graph.gray_graph);
    
    
    int n,x,y,z;
    f>>n;
    cout<<"Col: "<<col<<"  "<<"Row: "<<row<<endl;
    
    for(int i=0;i<n;i++){
        f>>x;
        f>>y;
        f>>z;
        
        int no = x * col + y;  // 'no' is the index of the node in vertex_set
        cout<<"No: "<<no<<"x,y: "<<vertex_set[no].x<<"  "<<vertex_set[no].y<<endl;
        
        if(z == 0){
            vertex_set[no].p = (-1)*col + (-1);
            super_s.adj.push_back(vertex_set[no]);  // super_s connected to s nodes  0
        }
        
        if(z == 1 ){
            vertex_set[no].adj.push_back(super_t);  // t nodes connected to super_t  1
        }
    }
    
    
    // use ff method
    Ford_Fulkerson(super_s, super_t,out_graph.gray_graph);
    
    return 0;
}// main end


void build_graph(Mat image) {
    
    // first, calulate node info and save it into the set
    int num = 0;
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < col; j++) {
            vertex node;
            node.x = i;
            node.y = j;
            if (i == 0) {
                if (j == 0) {
                    node.up = 0;
                    node.down = 510 - abs((int) image.at<uchar>(0, 0) - (int) image.at<uchar>(1, 0));
                    node.right = 510 -  abs((int) image.at<uchar>(0, 0) - (int) image.at<uchar>(0, 1));
                    node.left = 0;
                } else if (j == col - 1) {
                    node.up = 0;
                    node.down = 510 - abs((int) image.at<uchar>(i, j) - (int) image.at<uchar>(i + 1, j));
                    node.right = 0;
                    node.left = 510 - abs((int) image.at<uchar>(i, j) - (int) image.at<uchar>(i, j - 1));
                } else {
                    node.up = 0;
                    node.down = 510 - abs((int) image.at<uchar>(i, j) - (int) image.at<uchar>(i + 1, j));
                    node.right = 510 - abs((int) image.at<uchar>(i, j) - (int) image.at<uchar>(i, j + 1));
                    node.left = 510 - abs((int) image.at<uchar>(i, j) - (int) image.at<uchar>(i, j - 1));
                }
            } else if (i == row - 1) {
                if (j == 0) {
                    node.up = 510 - abs((int) image.at<uchar>(i, j) - (int) image.at<uchar>(i - 1, j));
                    node.down = 0;
                    node.right = 510 - abs((int) image.at<uchar>(i, j) - (int) image.at<uchar>(i, j + 1));
                    node.left = 0;
                } else if (j == col - 1) {
                    node.up = 510 - abs((int) image.at<uchar>(i, j) - (int) image.at<uchar>(i - 1, j));
                    node.down = 0;
                    node.right = 0;
                    node.left = 510 - abs((int) image.at<uchar>(i, j) - (int) image.at<uchar>(i, j - 1));
                } else {
                    node.up = 510 - abs((int) image.at<uchar>(i, j) - (int) image.at<uchar>(i - 1, j));
                    node.down = 0;
                    node.right = 510 - abs((int) image.at<uchar>(i, j) - (int) image.at<uchar>(i, j + 1));
                    node.left = 510 - abs((int) image.at<uchar>(i, j) - (int) image.at<uchar>(i, j - 1));
                }
            } else {
                if (j == 0) {
                    node.up = 510 - abs((int) image.at<uchar>(i, j) - (int) image.at<uchar>(i - 1, j));
                    node.down = 510 - abs((int) image.at<uchar>(i, j) - (int) image.at<uchar>(i + 1, j));
                    node.right = 510 - abs((int) image.at<uchar>(i, j) - (int) image.at<uchar>(i, j + 1));
                    node.left = 0;
                } else if (j == col - 1) {
                    node.up = 510 - abs((int) image.at<uchar>(i, j) - (int) image.at<uchar>(i - 1, j));
                    node.down = 510 - abs((int) image.at<uchar>(i, j) - (int) image.at<uchar>(i + 1, j));
                    node.right = 0;
                    node.left = 510 - abs((int) image.at<uchar>(i, j) - (int) image.at<uchar>(i, j - 1));
                } else {
                    node.up = 510 - abs((int) image.at<uchar>(i, j) - (int) image.at<uchar>(i - 1, j));
                    node.down = 510 - abs((int) image.at<uchar>(i, j) - (int) image.at<uchar>(i + 1, j));
                    node.right = 510 - abs((int) image.at<uchar>(i, j) - (int) image.at<uchar>(i, j + 1));
                    node.left = 510 - abs((int) image.at<uchar>(i, j) - (int) image.at<uchar>(i, j - 1));
                }
            }
            vertex_set.push_back(node);
            num++;
        }
    }
    cout << "Size: " << vertex_set.size() << ' ' << "Num " << num<<endl;
    
    // second, add adj info into all_adj
    int p = 0;
    vector<vertex> adj;
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < col; j++) {
            if (i == 0) {
                if (j == 0) {
                    adj.push_back(vertex_set[p + 1]);
                    adj.push_back(vertex_set[p + col]);
                } else if (j == col - 1) {
                    adj.push_back(vertex_set[p - 1]);
                    adj.push_back(vertex_set[p + col]);
                } else {
                    adj.push_back(vertex_set[p - 1]);
                    adj.push_back(vertex_set[p + 1]);
                    adj.push_back(vertex_set[p + col]);
                }
            } else if (i == row - 1) {
                if (j == 0) {
                    adj.push_back(vertex_set[p + 1]);
                    adj.push_back(vertex_set[p - col]);
                } else if (j == col - 1) {
                    adj.push_back(vertex_set[p - 1]);
                    adj.push_back(vertex_set[p - col]);
                } else {
                    adj.push_back(vertex_set[p - 1]);
                    adj.push_back(vertex_set[p + 1]);
                    adj.push_back(vertex_set[p - col]);
                    
                }
            } else {
                if (j == 0) {
                    adj.push_back(vertex_set[p - col]);
                    adj.push_back(vertex_set[p + col]);
                    adj.push_back(vertex_set[p + 1]);
                } else if (j == col - 1) {
                    adj.push_back(vertex_set[p - col]);
                    adj.push_back(vertex_set[p + col]);
                    adj.push_back(vertex_set[p - 1]);
                } else {
                    adj.push_back(vertex_set[p - col]);
                    adj.push_back(vertex_set[p + col]);
                    adj.push_back(vertex_set[p - 1]);
                    adj.push_back(vertex_set[p + 1]);
                }
            }
            p++;
            //            cout<<"adj size: "<<adj.size()<<endl;
            all_adj.push_back(adj);
            adj.clear();
            //            cout<<p<<endl;
        }
    }
    
    // give vertex.adj value
    for(int i=0;i<vertex_set.size();i++){
        vector<vertex> *p = &all_adj[i];
        vertex_set[i].adj = *p;
    }
    
    cout<<"All adj: "<<all_adj.size()<<endl;
    //    cout<<"Not visited: "<<vertex_set[700].visited<<endl;
    
    
    // ---------test for the correctness---------
    
    //    for(int i=0;i<vertex_set.size();i++){
    //        vertex node = vertex_set.at(i);
    //        cout<<"X: "<<node.x<<" Y: "<<node.y<<endl;
    //        cout<<"Up: "<<node.up<<" Down: "<<node.down<<" Right: "<<node.right<<" Left: "<<node.left<<endl;
    //        cout<<"Adj size: "<<node.adj.size()<<endl;
    //        for(int j=0;j<node.adj.size();j++){
    //            cout<<"Adj "<<j<<": "<<node.adj.at(j).x<<' '<<node.adj.at(j).y<<endl;
    //        }
    //    }
    
    
} //build edge end
bool bfs(vector<vertex> res_Graph, vertex super_s,vertex super_t,vector<vertex> &path, vector<int> &path2){
    cout<<"Enter bfs!"<<endl;
    vertex_set.push_back(super_s);  // add super s
    vertex_set.push_back(super_t);
    vertex path_last_label;
    queue<vertex> q;
    q.push(super_s);
    int n = vertex_set.size();
    bool visited[n+1];
    memset(visited, false, sizeof(visited));
    int parent[n+1];
    memset(parent, -1, sizeof(parent));
    int x = super_s.adj[0].x;
    int y = super_s.adj[0].y;
    int no = x * col + y;
    cout<<endl;
    int node_num = 0;
    while(!q.empty()){
        vertex cur = q.front();
        cout<<"cur node "<<cur.x<<"  "<<cur.y<<" adj size "<<cur.adj.size()<<endl;
        
        q.pop();
        
        for(int i=0;i<cur.adj.size();i++){
            
            if(cur.adj[i].x == -2){
                path_last_label = cur;
                vertex node = cur;
                
                int x = node.x;
                int y = node.y;
                int no = x * col + y;
                while(no != (-1) * col + (-1)){
                    path2.push_back(no);
                    no = vertex_set[no].p;
                    cout << "no = " << no << endl;
                    if (no == (-1) * col + (-1)) break;
                    node = vertex_set[no];
                }
                
                for(int i=path2.size()-1;i>=0;--i){
                    path.push_back(vertex_set[path2[i]]);
                }
                cout<<"Path size: "<<path.size()<<endl;
                return true;
            }
            int weight;
            
            // check direction and weight
            if(cur.x == -1){
                weight = 100000;  // cur is the super_s now
                cout<<"super_s!"<<endl;
            } else{
                if(cur.x == cur.adj[i].x){
                    if(cur.y > cur.adj[i].y) weight = cur.left;
                    else weight = cur.right;
                }
                if(cur.y == cur.adj[i].y){
                    if(cur.x < cur.adj[i].x) weight = cur.down;
                    else weight = cur.up;
                }
            }
            int x = cur.adj[i].x;
            int y = cur.adj[i].y;
            int no = x * col + y;
            
            if(visited[no] == false && weight > 0){
                cout<<"Next node: "<<x<<" "<<y<<"No: "<<no<<endl;
                q.push(vertex_set[no]);
                visited[no] = true;
                int xx = cur.x;
                int yy = cur.y;
                cout << "no : " << no << " cur.x : " << cur.x << " cur.y : " << cur.y << endl;
                int no2 = xx * col + yy;
                vertex_set[no].p = no2;            // store parent node
            }
        }
    }
    return false;
}

int get_min_cap(vector<vertex> res_Graph,vector<vertex> &path){
    int cap = INT_MAX;
    for(int i=0;i<path.size()-1;i++){
        vertex temp1 = path[i];
        int x1 = temp1.x;
        int y1 = temp1.y;
        int no1 = x1 * col + y1;
        vertex temp2 = path[i+1];
        int x2 = temp2.x;
        int y2 = temp2.y;
        int no2 = x2 * col + y2;
        if(temp1.x == temp2.x){
            if(temp1.y > temp2.y){
                if(cap > temp1.left) cap = vertex_set[no1].left;
            } else{
                if(cap > temp1.right) cap = vertex_set[no1].right;
            }
        }
        if(temp1.y == temp2.y){
            if(cap > temp2.x) cap = vertex_set[no1].up;
        } else{
            if(cap > temp1.down) cap = vertex_set[no1].down;
        }
    }
    return cap;
}

void update_res_graph(int &bottleNeck,vector<vertex> &path,vector<vertex> res_Graph){
    for(int i=0;i<path.size()-1;i++){
        // from temp1 to temp2
        vertex temp1 = path[i];
        int x1 = temp1.x;
        int y1 = temp1.y;
        int no1 = x1 * col + y1;
        vertex temp2 = path[i+1];
        int x2 = temp2.x;
        int y2 = temp2.y;
        int no2 = x2 * col + y2;
        if(temp1.x == temp2.x){
            if(temp1.y > temp2.y){  // 1 is at right of 2
                vertex_set[no1].left = vertex_set[no1].left - bottleNeck;
                vertex_set[no2].right = vertex_set[no2].right + bottleNeck;
                cout<<"here1"<<endl;
            } else{                 // 2 is at right of 1
                vertex_set[no1].right = vertex_set[no1].right - bottleNeck;
                vertex_set[no2].left = vertex_set[no2].left + bottleNeck;
                cout<<"here2"<<endl;
            }
        }
        if(temp1.y == temp2.y){
            if(temp1.x > temp2.x){  // 1 is down of 2
                vertex_set[no1].up = vertex_set[no1].up - bottleNeck;
                vertex_set[no2].down += bottleNeck;
                cout<<"here3"<<endl;
                
            }else{                  // 2 is down of 1
                vertex_set[no1].down-= bottleNeck;
                vertex_set[no2].up+=bottleNeck;
                cout<<"here4"<<endl;
            }
        }
    }
}
void bfs_ground(vector<vertex> ground,vertex node){
    queue<vertex> q;
    q.push(node);
    bool visited[vertex_set.size()];
    memset(visited, false, sizeof(visited));
    while(!q.empty()){
        vertex cur = q.front();
        q.pop();
        int x = cur.x;
        int y = cur.y;
        int n = x * col + y;
        ground.push_back(cur);   // add to back or foreground
        visited[n] = true;
        int weight = 0;
        for(int i=0;i<cur.adj.size();i++){
            int x = cur.adj[i].x;
            int y = cur.adj[i].y;
            int n = x * col + y;
            
            if(cur.x == cur.adj[i].x){
                if(cur.y > cur.adj[i].y) weight = cur.left;
                else weight = cur.right;
            }
            if(cur.y == cur.adj[i].y){
                if(cur.x > cur.adj[i].x) weight = cur.down;
                else weight = cur.up;
            }
            if(visited[n] == false && weight > 0){
                q.push(cur.adj[i]);
                visited[n] = true;
            }
        }
    }
}
void min_cut(vector<vertex> s,vector<vertex> t,vertex super_s,vertex super_t){
    for(int i=0;i<super_s.adj.size();i++){
        bfs_ground(s,super_s.adj[i]);
    }
    for(int j=0;j<super_t.adj.size();j++){
        bfs_ground(t,super_s.adj[j]);
    }
}
void Ford_Fulkerson(vertex super_s,vertex super_t,Mat image){
    res_Graph = vertex_set;   // get a copy from vertex_set
    vector<vertex> path;  // used in BFS to save the path
    vector<int> path2;
    while(bfs(res_Graph,super_s,super_t,path, path2)){
        int bottleNeck = get_min_cap(res_Graph,path);
        cout<<"bottleneck: "<<bottleNeck<<endl;
        update_res_graph(bottleNeck,path,res_Graph);
        path.clear();
        path2.clear();
    }
    // bfs from super_s and super_t to get s and t
    min_cut(s,t,super_s,super_t);
    
}

