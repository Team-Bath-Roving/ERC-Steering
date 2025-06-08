#include <map>
#include <vector>

#include "Arduino.h"

#include "stdlib.h"

// struct engineState {

//     std::map<int,bool> inj1 = {
//         {10,true},
//         {50,false}
//     };
//     std::map<int,bool> inj2 = {
//         {10,true},
//         {50,false}
//     };

// };

class Pin {
public:
    void set(bool state) {

    }
};

#define NUM_PINS 2

Pin inj1;
Pin inj2;

Pin * pins []= {
    &inj1,&inj2
};
enum pinNumbers{
    INJ1,INJ2
};

bool states [400][NUM_PINS] = {
    {true,false}, // 0
    {false,true},
};

bool initialStates[400][NUM_PINS] = {0};

void setStates(int step) {
    for (int i=0; i<NUM_PINS;i++) {
        pins[i]->set(states[i]);
    }
}