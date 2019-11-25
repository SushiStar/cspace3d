/*
 * All Rights Reserved.
 * 
 * This little program is used to extract planning info from the log file
 * From planning program.
 *
 * Planning time, number of expansions, epsilon value, solution cost, are
 * appended to the end of the designated file.
 * Run the program with the the following format:
 *
 *              executable <logfile_name> <document_name>
 *
 * Author: Wei Du
 * Date: 09/26/2018
 * Version: 2.0
 */


#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>

int main(int argc, char *argv[]) {
    std::ifstream logfile;
    std::ofstream document; 
    std::string line;

    logfile.open(argv[1]);
    if(!logfile.is_open()) {
        std::cout << "logfile does not exist @_@ " << std::endl;
        return 0;
    }
    std::string mapname(argv[1]);

    mapname = mapname.substr(0, mapname.size() - 4);

    // document file does not have to be existing.
    document.open(argv[2], std::ios_base::app);


    double time;
    int samples, cost;
    while( getline(logfile, line) ){
        if(std::string::npos != line.find("Time: ")){
            auto m = line.find("s");
            time = atof(line.substr(6,m-6).c_str());

            getline(logfile, line);
            getline(logfile, line);
            if(std::string::npos != line.find("solution cost: ")){
                auto n = line.find(":");
                cost = atoi(line.substr(n+1).c_str());
            }else{
                cost = 1e5;
                break;
            }

            getline(logfile, line);
            if(std::string::npos != line.find("Number of samples: ")){
                auto n = line.find(":");
                samples = atoi(line.substr(n+1).c_str());
            } else {
                samples = 1e5;
                break;
            }
        }
    }

    document << samples << " " << cost << " " << time << std::endl;

    logfile.close();
    document.close();

    return 0;
}
