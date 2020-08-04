/*
 * Copyright (c) 2020, Redshift Innovations
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* 
 * File:   test_dijkstra.cpp
 * Author: Nick Otero
 * Website: www.redshiftinnovations.tech
 */

#define CATCH_CONFIG_MAIN
#include <catch.hpp>
#include <rift/algorithms/search/dijkstra.hpp>

using namespace rift;

struct OrderedStruct {
    OrderedStruct(int _value) : value(_value) {}
    std::strong_ordering operator<=>(OrderedStruct const& rhs) const = default;
    
private:
    int value;
};

TEST_CASE( "Dijkstra on integers", "[dijkstra]" ) {
    let neighborOf = [](int i) -> std::vector<int> { 
        return {i - 1, i + 1};
    };
    let costToGo = [](let l, let r) { return std::abs(l - r); };
    let testDijkstra = [&](int start, int goal) {
        return dijkstra(start, goal, neighborOf, costToGo);
    };
    
    auto result = testDijkstra(0, 0);
    REQUIRE(result);
    REQUIRE(result->size() == 1);
    CHECK(result->front() == 0);
    
    result = testDijkstra(0, 1);
    REQUIRE(result);
    REQUIRE(result->size() == 2);
    CHECK(0 == result->front());
    CHECK(1 == result->back());
    
    result = testDijkstra(1, 0);
    REQUIRE(result);
    REQUIRE(result->size() == 2);
    CHECK(1 == result->front());
    CHECK(0 == result->back());
    
    result = testDijkstra(0, -1);
    REQUIRE(result);
    REQUIRE(result->size() == 2);
    CHECK(result->front() == 0);
    CHECK(result->back() == -1);
    
    result = testDijkstra(-1, 0);
    REQUIRE(result);
    REQUIRE(result->size() == 2);
    CHECK(result->front() == -1);
    CHECK(result->back() == 0);
    
    result = testDijkstra(0, 100);
    REQUIRE(result);
    REQUIRE(result->size() == 101);
    for (int i = 0; i <= 100; ++i) {
        CHECK(i == (*result)[i]);
    }
}

TEST_CASE( "Node Arena type is correct", "[dijkstra]" ) {
    auto nodeArena = detail::makeNodeArena<int, int>(10);
    CHECK(std::is_same_v<detail::HashedArena<int, int>, decltype(nodeArena)>);
    auto nodeArena1 = detail::makeNodeArena<OrderedStruct, int>(10);
    CHECK(std::is_same_v<
        detail::OrderedArena<OrderedStruct, int>, decltype(nodeArena1)>);
}