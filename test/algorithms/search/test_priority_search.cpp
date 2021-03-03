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
 * File:   test_priority_search.cpp
 * Author: Nick Otero
 * Website: www.redshiftinnovations.tech
 */

#define CATCH_CONFIG_MAIN
#include <catch.hpp>
#include <rift/algorithms/search/priority_search.hpp>
#include <rift/util/zip.hpp>

using namespace rift;

struct OrderedStruct {
    OrderedStruct(int _value) : value(_value) {}
    std::strong_ordering operator<=>(OrderedStruct const& rhs) const = default;
    
private:
    int value;
};

TEST_CASE( "Priority Search on integers", "[priority_search]" ) {
    let neighborOf = [](int i) -> std::vector<int> { 
        return {i - 1, i + 1};
    };
    let costToGo = [](let l, let r) { return std::abs(l - r); };
    let testSearch = [&](int start, int goal) {
        return prioritySearch(start, goal, neighborOf, costToGo);
    };
    
    auto result = testSearch(0, 0);
    REQUIRE(result);
    REQUIRE(result->size() == 1);
    CHECK(result->front() == 0);
    
    result = testSearch(0, 1);
    REQUIRE(result);
    REQUIRE(result->size() == 2);
    CHECK(0 == result->front());
    CHECK(1 == result->back());
    
    result = testSearch(1, 0);
    REQUIRE(result);
    REQUIRE(result->size() == 2);
    CHECK(1 == result->front());
    CHECK(0 == result->back());
    
    result = testSearch(0, -1);
    REQUIRE(result);
    REQUIRE(result->size() == 2);
    CHECK(result->front() == 0);
    CHECK(result->back() == -1);
    
    result = testSearch(-1, 0);
    REQUIRE(result);
    REQUIRE(result->size() == 2);
    CHECK(result->front() == -1);
    CHECK(result->back() == 0);
    
    result = testSearch(0, 100);
    REQUIRE(result);
    REQUIRE(result->size() == 101);
    for (int i = 0; i <= 100; ++i) {
        CHECK(i == (*result)[i]);
    }
}

TEST_CASE( "Search on empty flat grid", "[priority_search]" ) {
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
    let testSearch = [&](
        unsigned int start, 
        unsigned int goal,
        std::vector<unsigned int> expected) {
        
        let maybePath = prioritySearch(start, goal, neighborFn, costFn);
        REQUIRE(maybePath);
        CHECK(maybePath->size() == expected.size());
        let zippedResult = zip(*maybePath, expected);
        for (let p : zippedResult) CHECK(p.first == p.second); 
    };
    testSearch(8, 8, {8});
    testSearch(8, 9, {8, 9});
    testSearch(8, 12, {8, 9, 10, 11, 12});
    testSearch(8, 15, {8, 15});
    testSearch(8, 22, {8, 15, 22});
    testSearch(8, 36, {8, 15, 22, 29, 36});
    testSearch(8, 40, {8, 9, 10, 11, 18, 25, 32, 39, 40});
}

TEST_CASE( "Search on an occupied flat grid", "[dijkstra]" ) {
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
    let testSearch = [&](
        unsigned int start, 
        unsigned int goal,
        std::vector<unsigned int> expected) {
        
        let maybePath = prioritySearch(start, goal, neighborFn, costFn);
        REQUIRE(maybePath);
        CHECK(maybePath->size() == expected.size());
        let zippedResult = zip(*maybePath, expected);
        for (let p : zippedResult) CHECK(p.first == p.second); 
    };
    testSearch(8, 11, {8, 9, 16, 23, 22, 29, 36, 37, 38, 39, 40, 33, 26, 25, 18, 11});
    testSearch(11, 8, {11, 12, 19, 26, 33, 40, 39, 38, 37, 36, 29, 22, 23, 16, 9, 8});
}

TEST_CASE( "Search with dense iterator", "[priority_search]" ) {
    constexpr std::array grid = {
        'x', 'x', 'x', 'x', 'x', 'x', 'x',
        'x', 'a', 'b', 'c', 'd', 'e', 'x',
        'x', 'e', 'f', 'g', 'h', 'j', 'x',
        'x', 'i', 'j', 'k', 'l', 'o', 'x', 
        'x', 'm', 'n', 'o', 'p', 't', 'x',
        'x', 'u', 'v', 'w', 'y', 'z', 'x',
        'x', 'x', 'x', 'x', 'x', 'x', 'x'
    };
    let neighborFn = [&](auto const& i) {
        std::vector<Index> result;
        result.push_back(i + 1);
        result.push_back(i + 7);
        if (i >= 1) result.push_back(i - 1);
        if (i >= 7) result.push_back(i - 7);
        return result;
    };
    let costFn = [&](auto const& from, auto const& to) {
        REQUIRE(to < grid.size());
        let value = grid[to.index];
        if (value == 'x') return std::numeric_limits<unsigned char>::max();
        else return static_cast<unsigned char>(1);
    };
    let testSearch = [&](
        Index start, 
        Index goal,
        std::vector<Index> expected) {
        
        let maybePath = prioritySearch(start, goal, neighborFn, costFn);
        REQUIRE(maybePath);
        CHECK(maybePath->size() == expected.size());
        let zippedResult = zip(*maybePath, expected);
        for (let p : zippedResult) CHECK(p.first == p.second); 
    };
    std::vector<size_t> expected = {8, 9, 10, 11, 18, 25, 32, 39, 40};
    std::vector<Index> expectedIndices;
    std::ranges::transform(expected, std::back_inserter(expectedIndices), [](auto i){return Index(i);});
    testSearch(Index(8), Index(40), expectedIndices);
}

TEST_CASE( "Node Arena type is correct", "[priority_search]" ) {
    auto nodeArena = detail::makeNodeArena<int, int>(10);
    CHECK(std::is_same_v<detail::HashedArena<int, int>, decltype(nodeArena)>);
    auto nodeArena1 = detail::makeNodeArena<OrderedStruct, int>(10);
    CHECK(std::is_same_v<
        detail::OrderedArena<OrderedStruct, int>, decltype(nodeArena1)>);
    auto nodeArena2 = detail::makeNodeArena<Index, int>(10);
    CHECK(std::is_same_v<detail::DenseArena<Index, int>, decltype(nodeArena2)>);
}