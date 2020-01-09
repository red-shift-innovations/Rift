#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "rift/numeric/bounded.hpp"

using namespace rift;

TEST_CASE( "Bounded numbers are constructable", "[bounded]" ) {
    CHECK_NOTHROW(Bounded<int, 0, 1>(0));
    CHECK_NOTHROW(Bounded<int, 0, 1>(1));
    CHECK_NOTHROW(Bounded<int, -5, 5>(0));
    CHECK_NOTHROW(Bounded<int, -5, 5>(4));
    CHECK_NOTHROW(Bounded<int, -5, 5>(-4));

    CHECK_NOTHROW(Bounded<float, -1, 1>(0.0f));
    CHECK_NOTHROW(Bounded<float, -1, 1>(0.5f));
    CHECK_NOTHROW(Bounded<float, -1, 1>(-0.5f));
}

TEST_CASE( "Bounded numbers fail constructed out of bounds", "[bounded]" ) {
    CHECK_THROWS(Bounded<int, 0, 1>(-1));
    CHECK_THROWS(Bounded<int, 0, 1>(2));

    CHECK_THROWS(Bounded<float, 0, 1>(-1));
    CHECK_THROWS(Bounded<float, 0, 1>(2));
    CHECK_THROWS(Bounded<float, 0, 1>(-0.1));
    CHECK_THROWS(Bounded<float, 0, 1>(1.1));
}