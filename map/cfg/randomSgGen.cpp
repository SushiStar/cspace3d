/**
 * Randomly generate start and goal pair.
 *
 */
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cassert>
#include <cstdlib>
#include <ctime>

class pt{
public:
    pt(): x(0), y(0){}
    int x;
    int y;
};

class themap{
public:
    themap(): width(0), height(0), resolution(0.025){
        std::srand(std::time(nullptr));
    }
    ~themap(){};
    void read_map(char* input_);
    void output_sg();

private:
    int width, height;
    double resolution;
    std::ifstream input;
    std::ofstream output;
    std::string inputname;
    std::vector<int> mapdata;
    pt s,g;

    void generate_GS();

};

void themap::read_map(char* input_) {
    inputname = input_;
    std::string buf;

    input.open(input_);
    if(!input.is_open()) {
        std::cout << "File cannot be opened: " << inputname << std::endl;
    }
    
    // Get the size of the map
    getline(input, buf);
    std::istringstream iss(buf);

    /*
     *auto pos1 = buf.rfind(' ');
     *auto pos2 = buf.find(':');
     *assert(std::string::npos != pos1 &&
     *       std::string::npos != pos2 );
     *std::string number1 = buf.substr(pos1);
     *std::string number2 = buf.substr(pos2+1,pos1-pos2);
     */
    std::string number1, number2;
    iss >> number1;
    iss >> number1;
    iss >> number2;

    // height; width
    width = std::stoi(number1);
    height = std::stoi(number2);
    mapdata.reserve(height * width);

    // dump the useless
    for(int i = 0; i < 9; ++i )
        getline(input, buf);

    while(getline(input, buf)){
        auto temp = buf.c_str();
        for (int i = 0; i < buf.size(); ++i) {
            mapdata.push_back(atoi(&temp[i]));    
            ++i;
        }
    }

    input.close();
}

void themap::generate_GS() {
    // break the map into nxn cell
    
    // 4 vertices of the center square
    // leftdown, rightdown, leftup, rightup
    pt pld,prd,plu,pru;
    pld.x = (int)(width/3.0 + 2);
    pld.y = (int)(height/3.0 + 2);

    prd.x = (int)(width/3.0*2.0 - 2);
    prd.y = (int)(height/3.0 + 2);

    plu.x = (int)(width/3.0 + 2);
    plu.y = (int)(height/3.0*2.0 - 2);
    
    pru.x = (int)(width/3.0*2.0  - 2);
    pru.y = (int)(height/3.0*2.0 - 2);
 
    bool infeasible = true;
    while( infeasible ) {

        // odd number ld->ur;
        // even number rd->lu;
        int dice = std::rand()%2;
        int x1,x2,y1,y2;

        int w_r = (int) (width/3.0);
        int h_r = (int) (height/3.0);

        x1 = std::rand()%w_r;
        x2 = std::rand()%h_r;
        y1 = std::rand()%w_r;
        y2 = std::rand()%h_r;

        if(dice) {  //ld->ur
            s.x = pld.x - x1;
            s.y = pld.y - y1;
            g.x = pru.x + x2; 
            g.y = pru.y + y2;
        } else {    // rd->lu;
            s.x = plu.x - x1;
            s.y = plu.y + y1;
            g.x = prd.x + x2; 
            g.y = prd.y - y2;
        }

        if (s.x + s.y * width >= mapdata.size() ||
            g.x + g.y * width>= mapdata.size())
            continue;

        // center
        int f1 = mapdata.at(s.x + s.y * width);
        int f2 = mapdata.at(g.x + g.y * width);
        if(0 == f1 &&  0 == f2 ){
            infeasible = false;
        } 
    }
}

void themap::output_sg() {
    generate_GS();
    std::string outputname(inputname);
    outputname.replace(outputname.end()-3, outputname.end()-1, 1, 's');

    output.open(outputname.c_str());
    output << "start: " << s.x * resolution << " "<< s.y * resolution << " 0" << std::endl;
    output << "goal: " << g.x * resolution << " "<< g.y * resolution << " 0" <<std::endl;
    output << "radius: 4.0" << std::endl;

    output.close();
}


int main (int argc, char* argv[]) {
    themap thismap;
    thismap.read_map(argv[1]);
    thismap.output_sg();
    return 0;
}

