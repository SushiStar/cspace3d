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
 * Version: 1.0
 */

#include<fstream>
#include<iostream>
#include<cstdlib>

int main(int argc, char *argv[]) {
    std::ifstream logfile;
    std::ofstream document; 
    std::string line;
    const std::string extract("time=");

    logfile.open(argv[1]);
    if(!logfile.is_open()) {
        std::cout << "logfile does not exist @_@ " << std::endl;
        return 0;
    }
    std::string mapname(argv[1]);

    mapname = mapname.substr(0, mapname.size() - 4);

    // document file does not have to be existing.
    document.open(argv[2], std::ios_base::app);

    while(getline(logfile,line)){
        auto n = line.find(extract);
        auto m = line.substr(n+5, line.size());
        document << m << std::endl;
    }   


    logfile.close();
    document.close();

    return 0;
}
