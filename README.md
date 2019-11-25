# X,Y,$\theta$ planning in continuous space with soft duplicate detection.

### About soft duplicate detection
Please refer to this paper: 

### Dependencies:
This package is only dependent on the sbpl library.
So install the sbpl library first: based on [this](https://github.com/sbpl/sbpl) repo, which is from the Search-based planning Lab.

### Run this program:
To build this program:
```
mkdir build && cd build
cmake ../ && make
```
To run the planner:
```
./xythetac <mapname> <start&goal> <motion_primitive>
```
The \<filename\> is replaced by the map filename, which is in the format```(*.cfg)``` the same as the one in the sbpl library.
The \<start&goal\> is the start and goal setup file, which is in ```*.sg``` format.
The \<motion_primitive\> is in format ```*.mprim```, which could be generated from the sbpl library.




