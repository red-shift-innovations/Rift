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
#include <rift/util/zip.hpp>

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

TEST_CASE( "Dijkstra on empty flat grid", "[dijkstra]" ) {
    constexpr std::array grid = {
        'x', 'x', 'x', 'x', 'x', 'x', 'x',
        'x', 'a', 'b', 'c', 'd', 'e', 'x',
        'x', 'e', 'f', 'g', 'h', 'j', 'x',
        'x', 'i', 'j', 'k', 'l', 'o', 'x', 
        'x', 'm', 'n', 'o', 'p', 't', 'x',
        'x', 'u', 'v', 'w', 'y', 'z', 'x',
        'x', 'x', 'x', 'x', 'x', 'x', 'x'
    };
    let neighborFn = [](unsigned int i) {
        std::vector<unsigned int> result;
        result.push_back(i + 1);
        result.push_back(i + 7);
        if (i >= 1) result.push_back(i - 1);
        if (i >= 7) result.push_back(i - 7);
        return result;
    };
    let costFn = [&grid](unsigned int f, unsigned int t) {
        REQUIRE(t < grid.size());
        let value = grid[t];
        if (value == 'x') return std::numeric_limits<unsigned char>::max();
        else return static_cast<unsigned char>(1);
    };
    let testDijkstra = [&](
        unsigned int start, 
        unsigned int goal,
        std::vector<unsigned int> expected) {
        
        let maybePath = dijkstra(start, goal, neighborFn, costFn);
        REQUIRE(maybePath);
        CHECK(maybePath->size() == expected.size());
        let zippedResult = zip(*maybePath, expected);
        for (let p : zippedResult) CHECK(p.first == p.second); 
    };
    testDijkstra(8, 8, {8});
    testDijkstra(8, 9, {8, 9});
    testDijkstra(8, 12, {8, 9, 10, 11, 12});
    testDijkstra(8, 15, {8, 15});
    testDijkstra(8, 22, {8, 15, 22});
    testDijkstra(8, 36, {8, 15, 22, 29, 36});
    testDijkstra(8, 40, {8, 9, 10, 11, 18, 25, 32, 39, 40});
}

TEST_CASE( "Dijkstra on an occupied flat grid", "[dijkstra]" ) {
    constexpr std::array grid = {
        'x', 'x', 'x', 'x', 'x', 'x', 'x',
        'x', 'a', 'b', 'x', 'd', 'e', 'x',
        'x', 'x', 'f', 'x', 'h', 'j', 'x',
        'x', 'i', 'j', 'x', 'l', 'o', 'x', 
        'x', 'm', 'x', 'x', 'x', 'n', 'x',
        'x', 'u', 'v', 'w', 'y', 'z', 'x',
        'x', 'x', 'x', 'x', 'x', 'x', 'x'
    };
    let neighborFn = [](unsigned int i) {
        std::vector<unsigned int> result;
        result.push_back(i + 1);
        result.push_back(i + 7);
        if (i >= 1) result.push_back(i - 1);
        if (i >= 7) result.push_back(i - 7);
        return result;
    };
    let costFn = [&grid](unsigned int f, unsigned int t) {
        REQUIRE(t < grid.size());
        let value = grid[t];
        if (value == 'x') return std::numeric_limits<unsigned char>::max();
        else return static_cast<unsigned char>(1);
    };
    let testDijkstra = [&](
        unsigned int start, 
        unsigned int goal,
        std::vector<unsigned int> expected) {
        
        let maybePath = dijkstra(start, goal, neighborFn, costFn);
        REQUIRE(maybePath);
        CHECK(maybePath->size() == expected.size());
        let zippedResult = zip(*maybePath, expected);
        for (let p : zippedResult) CHECK(p.first == p.second); 
    };
    testDijkstra(8, 11, {8, 9, 16, 23, 22, 29, 36, 37, 38, 39, 40, 33, 26, 25, 18, 11});
    testDijkstra(11, 8, {11, 12, 19, 26, 33, 40, 39, 38, 37, 36, 29, 22, 23, 16, 9, 8});
}

TEST_CASE( "Node Arena type is correct", "[dijkstra]" ) {
    auto nodeArena = detail::makeNodeArena<int, int>(10);
    CHECK(std::is_same_v<detail::HashedArena<int, int>, decltype(nodeArena)>);
    auto nodeArena1 = detail::makeNodeArena<OrderedStruct, int>(10);
    CHECK(std::is_same_v<
        detail::OrderedArena<OrderedStruct, int>, decltype(nodeArena1)>);
}