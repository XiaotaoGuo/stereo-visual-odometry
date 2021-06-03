#include <gtest/gtest.h>

#include "test_algorithm.hpp"
// #include "test_image_reader.hpp"
#include "test_map_visualizer.hpp"

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}