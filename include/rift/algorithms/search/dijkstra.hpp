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
 * File:   dijkstra.hpp
 * Author: Nick Otero
 * Website: www.redshiftinnovations.tech
 */

#pragma once

#include <algorithm>
#include <concepts>
#include <limits>
#include <optional>
#include <type_traits>
#include <vector>

#include <rift/algorithms/search/search_concepts.hpp>
#include <rift/util/rustify.hpp>

namespace rift {
namespace detail {
    
    /**
     * Struct used to encode a node in a graph being searched using Dijkstra's
     * shortest path algorithm
     * 
     * \tparam T an equality comparable type encoding the domain of the search
     * \tparam C an arithmetic type used to encode edge costs
     */
    template <std::equality_comparable T, typename C>
    requires std::is_arithmetic_v<C>
    struct DijkstraNode {
        /**
         * Constructor
         */
        DijkstraNode(T dataIn) : data(dataIn) {}

        /** The data represented by this node */
        T data;

        /** The parent of this node */
        std::shared_ptr<DijkstraNode<T, C>> parent;

        /** The accumulated cost to come to this node */
        C costToCome = std::numeric_limits<C>::max();

        /** Flag indicating if this node is on the open set */
        bool isOpen = false;

        /** Flag indicating if this node has been removed from the open set */
        bool isClosed = false;
    };
    /** Utility type alias for shared_ptrs of type DijkstraNode */
    template <typename T, typename C>
    using DijkstraNodePtr = std::shared_ptr<DijkstraNode<T, C>>;;
    
    /** Utility stuct for base case predicate to determine if a type is hashable 
      * using std::hash */
    template <typename T, typename = std::void_t<>>
    struct is_std_hashable : std::false_type { };

    /** Meta function used to determine if a given struct is hashable using
     * std::hash
     */
    template <typename T>
    struct is_std_hashable<T, std::void_t<
        decltype(std::declval<std::hash<T>>()(std::declval<T>()))>> 
    : std::true_type { };

    /**
     * A type is Hashable if it has an implementation for std::hash<T>. This is
     * tested using the meta function is_std::hashable<T>
     */
    template <typename T>
    concept Hashable = is_std_hashable<T>::value; 
    
    /**
     * A HashedArena is a std::unordered_map which maps from the user's domain
     * to the Dijkstra search graph domain
     */
    template <Hashable T, typename C>
    using HashedArena = 
        std::unordered_map<T, DijkstraNodePtr<T, C>>;
    
    /**
     * An OrderedArena is a std::map which maps from the user's domain
     * to the Dijkstra search graph domain
     */
    template <typename T, typename C>
    using OrderedArena = 
        std::map<T, DijkstraNodePtr<T, C>>;
    
    /**
     * Make a HashedArena to be used for the Dijkstra search
     * 
     * Node arenas are required to hold instances of nodes used by the graph
     * created during the Dijkstra search. This factory function creates a
     * HashedArena if its template argument satifies the requirements of
     * Hashable.
     * 
     * \tparam T The user's hashable domain type
     * \tparam C The type encoding edge costs within the graph
     * \param sizeHint A size to initialize the arena with for optimal 
     *        allocations
     *
     * \return The created HashedArena
     */
    template <Hashable T, typename C>
    fn makeNodeArena(size_t sizeHint) -> HashedArena<T, C> {
        HashedArena<T, C> result;
        result.reserve(sizeHint);
        return result;
    }
    
    /**
     * Make an OrderedArena to be used for the Dijkstra search
     * 
     * Node arenas are required to hold instances of nodes used by the graph
     * created during the Dijkstra search. This factory function creates an
     * OrderedArena if its template argument satifies the requirements of
     * std::strict_weak_order for std::less<T>. Note that HashedArenas are more
     * efficient than OrderedArenas, so this factory function requires that it
     * be discarded if the Hashed version is satisfied.
     * 
     * \tparam T The user's weakly orderable domain type
     * \tparam C The type encoding edge costs within the graph
     * \param sizeHint A size to initialize the arena with for optimal 
     *        allocations
     *
     * \return The created OrderedArena
     */
    template <typename T, typename C>
    requires (!Hashable<T> && std::strict_weak_order<std::less<T>, T, T>)
    fn makeNodeArena(size_t sizeHint) -> OrderedArena<T, C> {
        OrderedArena<T, C> result;
        return result;
    }

    /**
     * The state of an ongoing Dijkstra search
     * 
     * \tparam T an equality comparable type encoding the domain of the search
     * \tparam C an arithmetic type encoding the edge cost within the graph
     * \tparam Arena the type of arena used to store DijkstraNode instances
     *         generated during the search
     */
    template <std::equality_comparable T, typename C, typename Arena>
    struct DijkstraState {
        /** The arena used to store nodes generated during the search */
        Arena nodeArena;
        
        /** The set of open nodes to be expanded for the search */
        std::vector<DijkstraNodePtr<T, C>> openSet;
    };
}
        
    /**
     * Perform Dijkstra's Shortest Paths algorithm
     * 
     * Given a start and goal in a domain provided by the user, find the
     * shortest path between them. Neighbors of data within the user's domain
     * are determined using the provided neighborFn function. The cost to go
     * from one datum to another is determined using the provided costFn
     * function. An optional size hint can be provided for optimal allocations.
     * 
     * \tparam T an equality comparable type encoding the domain of the search
     * \tparam N an invocable which returns a vector<T> of neighbors
     * \tparam G an invocable which returns an aritmetic type encoding edge cost
     * \param start the starting point of the search
     * \param goal the goal point of the search
     * \param neighborFn the invocable which provides neighbors of the user's
     *        data
     * \param costFn the invocable which provides edge costs between the user's
     *        data
     * \param sizeHint a size hint used to optimize allocations
     * 
     * \return A std::optional containing std::nullopt if no path is found, or
     *         a vector of data within the user's domain forming a path from
     *         start to goal
     */
    template <std::equality_comparable T, NeighborFn<T> N, CostFn<T> G>
    fn dijkstra(T start, T goal, N neighborFn, G costFn, size_t sizeHint = 1000) 
        -> std::optional<std::vector<T>> {
        
        using Cost = std::invoke_result_t<G, T, T>;
        auto arena = detail::makeNodeArena<T, Cost>(sizeHint);
        using State = detail::DijkstraState<T, Cost, decltype(arena)>;
        using Node = detail::DijkstraNode<T, Cost>;
        using NodePtr = detail::DijkstraNodePtr<T, Cost>;
        std::vector<T> result;
        State searchState;
        let sortNodes = [](let l, let r) {return l->costToCome > r->costToCome;};
        let push = [&openSet = searchState.openSet, &sortNodes](NodePtr n) {
            // Don't push nodes if they have already been closed
            if (n->isClosed) return;
            if (n->isOpen) {
                // If the node is in the open set, find it.
                auto it = std::ranges::find_if(
                    openSet, [&n](let f) { return n->data == f->data; });
                if (it == end(openSet)) {
                    // This shouldn't happen, but in case it does...
                    goto pushNew;
                } else {
                    // The node in the open set has been found. Revise its cost
                    // and resort the heap.
                    (*it)->costToCome = n->costToCome;
                    std::make_heap(begin(openSet), end(openSet), sortNodes);
                }
            } else {
                // If the node is not in the open set, mark it as open and push
                // it to the heap.
                pushNew:
                n->isOpen = true;
                openSet.push_back(n);
                std::push_heap(begin(openSet), end(openSet), sortNodes);
            }
        };
        let pop = [&openSet = searchState.openSet, &sortNodes] {
            std::pop_heap(begin(openSet), end(openSet), sortNodes);
            NodePtr node = openSet.back();
            openSet.pop_back();
            node->isClosed = true;
            return node;
        };
        
        // Special case, check if the start and the goal are the same.
        if (start == goal) {
            result.push_back(start);
            return result;
        }
        
        auto [startIt, startInserted] = searchState.nodeArena.emplace(
            start, std::make_shared<Node>(start));
        Node goalNode(goal);
        startIt->second->costToCome = 0;
        searchState.openSet.push_back(startIt->second);
        while (!searchState.openSet.empty()) {
            let currentNode = pop();
            if (currentNode->data == goal) {
                goalNode = *currentNode;
                break;
            }
            let neighbors = neighborFn(currentNode->data);
            for (let neighbor : neighbors) {
                auto [it, inserted] = searchState.nodeArena.emplace(
                    neighbor, std::make_shared<Node>(neighbor));
                NodePtr neighborNode = it->second;
                                          
                let newCost = currentNode->costToCome 
                    + costFn(currentNode->data, neighbor);
                if (newCost == std::numeric_limits<Cost>::max()) continue;
                if (newCost < neighborNode->costToCome) {
                    neighborNode->costToCome = newCost;
                    neighborNode->parent = currentNode;
                    push(neighborNode);
                }
            }
        }
        if (!goalNode.parent) {
            return std::nullopt;
        }
        
        result.push_back(goal);
        auto parent = goalNode.parent;
        while (parent && parent->data != start) {
//            std::cout << parent->data << std::endl;
            result.push_back(parent->data);
            parent = parent->parent;
        }
        if (!parent || parent->data != start) {
            return std::nullopt;
        }
        result.push_back(start);
        std::reverse(begin(result), end(result));
        return result;
    }
}

