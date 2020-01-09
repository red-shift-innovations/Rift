#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "rift/numeric/probability.hpp"

using namespace rift;

TEST_CASE( "Bounded numbers are constructable", "[bounded]" ) {
    CHECK_NOTHROW(Probability(0));
    CHECK_NOTHROW(Probability(1));
    CHECK_NOTHROW(Probability(0.5));
}

TEST_CASE( "Bounded numbers fail constructed out of bounds", "[bounded]" ) {
    CHECK_THROWS(Probability(-1));
    CHECK_THROWS(Probability(2));
    CHECK_THROWS(Probability(-0.1));
    CHECK_THROWS(Probability(1.1));
}