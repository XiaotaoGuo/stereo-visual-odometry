#include "config.h"
#include "Visual_odometry.h"

int main(int argc, char** argv){

    shared_ptr<Visual_odometry> vo = shared_ptr<Visual_odometry>(new Visual_odometry);
    vo->init(argv[1], argv[2]);

    assert(vo->Init() == true);
    vo->start();

    return 0;

}


