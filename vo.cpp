#include "Visual_odometry.h"
#include "config.h"

int main(int argc, char** argv) {
    shared_ptr<Visual_odometry> vo =
        shared_ptr<Visual_odometry>(new Visual_odometry);

    int sequence_id = stoi(argv[2]);
    assert(vo->init(argv[1], sequence_id) == true);
    vo->start();

    return 0;
}
