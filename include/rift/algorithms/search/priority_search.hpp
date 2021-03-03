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
 * File:   priority_search.hpp
 * Author: Nick Otero
 * Website: www.redshiftinnovations.tech
 */

#pragma once

#include <rift/algorithms/search/search_concepts.hpp>
#include <rift/util/concepts.hpp>
#include <rift/util/index.hpp>
#include <rift/util/type_traits.hpp>

#include <rift/util/rustify.hpp>

namespace rift {
namespace detail {
    
    /**
     * Struct used to encode a node in a graph being searched using a
     * shortest path algorithm
     * 
     * \tparam T an equality comparable type encoding the domain of the search
     * \tparam C an arithmetic type used to encode edge costs
     */
    template <std::equality_comparable T, typename C>
    requires std::is_arithmetic_v<C>
    struct SearchNode {
        /**
         * Constructor
         */
        SearchNode(T dataIn) : data(dataIn) {}

        /** The data represented by this node */
        T data;

        /** The parent of this node */
        std::shared_ptr<SearchNode<T, C>> parent;

        /** The accumulated cost to come to this node */
        C costToCome = std::numeric_limits<C>::max();

        /** Flag indicating if this node is on the open set */
        bool isOpen = false;

        /** Flag indicating if this node has been removed from the open set */
        bool isClosed = false;
    };
    
    /** Utility type alias for shared_ptrs of type SearchNode */
    template <typename T, typename C>
    using SearchNodePtr = std::shared_ptr<SearchNode<T, C>>;
    
    /**
     * A DenseArena is a vector of SearchNodePtrs.
     */
    template <typename T, typename C>
    using DenseArena = std::vector<SearchNodePtr<T, C>>;
    
    /**
     * A HashedArena is a std::unordered_map which maps from the user's domain
     * to the Dijkstra search graph domain
     */
    template <Hashable T, typename C>
    using HashedArena = 
        std::unordered_map<T, SearchNodePtr<T, C>>;
    
    /**
     * Utility concept to determine if the search domain is ordered.
     * Ordered is a weaker concept than Hashable, so don't test if if the type
     * is hashable.
     */
    template <typename T>
    concept Ordered = 
        !Hashable<T> 
        && std::strict_weak_order<std::less<T>, T, T>;
    
    /**
     * An OrderedArena is a std::map which maps from the user's domain
     * to the search graph domain
     */
    template <Ordered T, typename C>
    using OrderedArena = 
        std::map<T, SearchNodePtr<T, C>>;
    
    /**
     * Emplace a node into the arena.
     * 
     * \param container the container into which the key and value will be
     *        emplaced.
     * \param key the user domain value.
     * \param value the search node domain value.
     * 
     * \return the emplaced value.
     */
    template <typename C, typename K, typename V>
    fn emplace(C& container, K&& key, V&& value) -> V {
        auto [it, wasInserted] = container.emplace(
            std::make_pair(std::forward<K>(key), std::forward<V>(value)));
        return it->second;
    }
    
    /**
     * Emplace the given search node into the given dense arena.
     * 
     * \param arena the arena into which the value will be emplaced.
     * \param index the index at which the value will be emplaced.
     * \param value the value to emplace into the arena.
     * 
     * \return the emplaced value.
     */
    template <typename T>
    fn emplace(
        DenseArena<Index, T>& arena, 
        Index index, 
        typename DenseArena<Index, T>::value_type value) {
        
        if (index.index >= arena.size()) {
            arena.resize(index.index + 1);
        }
        let storedValue = arena[index.index];
        if (!storedValue) {
            arena[index.index] = value;
            return value;
        } else {
            return storedValue;
        }
    }
    
    /**
     * Make a HashedArena to be used for the search.
     * 
     * Node arenas are required to hold instances of nodes used by the graph
     * created during the search. This factory function creates a
     * HashedArena if its template argument satifies the requirements of
     * Hashable.
     * 
     * \tparam T The user's hashable domain type
     * \tparam C The type encoding edge costs within the graph
     * \param[in] sizeHint A size to initialize the arena with for optimal 
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
     * Make an OrderedArena to be used for the search.
     * 
     * Node arenas are required to hold instances of nodes used by the graph
     * created during the search. This factory function creates an
     * OrderedArena if its template argument satifies the requirements of
     * std::strict_weak_order for std::less<T>. Note that HashedArenas are more
     * efficient than OrderedArenas, so this factory function requires that it
     * be discarded if the Hashed version is satisfied.
     * 
     * \tparam T The user's weakly orderable domain type
     * \tparam C The type encoding edge costs within the graph
     * \param[in] sizeHint A size to initialize the arena with for optimal 
     *        allocations
     *
     * \return The created OrderedArena
     */
    template <Ordered T, typename C>
    requires (!std::same_as<std::decay_t<T>, Index>)
    fn makeNodeArena(size_t sizeHint) -> OrderedArena<T, C> {
        OrderedArena<T, C> result;
        return result;
    }
    
    /**
     * Make a DenseArena.
     * 
     * \tparam I The index type.
     * \tparam C The search node type.
     * \param sizeHint The initial size of the arena.
     * 
     * \return a DenseArena of search nodes.
     */
    template <typename I, typename C>
    requires std::same_as<I, std::decay_t<Index>>
    fn makeNodeArena(size_t sizeHint) -> DenseArena<Index, C> {
        DenseArena<Index, C> result(sizeHint);
        return result;
    }

    /**
     * The state of an ongoing search
     * 
     * \tparam T an equality comparable type encoding the domain of the search
     * \tparam C an arithmetic type encoding the edge cost within the graph
     * \tparam Arena the type of arena used to store SearchNode instances
     *         generated during the search
     */
    template <std::equality_comparable T, typename C, typename Arena>
    struct SearchState {
        /** The arena used to store nodes generated during the search */
        Arena nodeArena;
        
        /** The set of open nodes to be expanded for the search */
        std::vector<SearchNodePtr<T, C>> openSet;
    };
}

template <
    std::equality_comparable T, 
    NeighborFn<T> N,
    CostFn<T> G>
fn prioritySearch(T start, T goal, N neighborFn, G costFn, size_t sizeHint = 1000) 
    -> std::optional<std::vector<T>> {
    
    using Cost = std::invoke_result_t<G, T, T>;
    auto arena = detail::makeNodeArena<T, Cost>(sizeHint);
    using State = detail::SearchState<T, Cost, decltype(arena)>;
    using Node = detail::SearchNode<T, Cost>;
    using NodePtr = detail::SearchNodePtr<T, Cost>;
    std::vector<T> result;
    State searchState;
    searchState.nodeArena = std::move(arena);
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

    auto startNodePtr = emplace(
        searchState.nodeArena, start, std::make_shared<Node>(start));
    Node goalNode(goal);
    startNodePtr->costToCome = 0;
    searchState.openSet.push_back(startNodePtr);
    while (!searchState.openSet.empty()) {
        let currentNode = pop();
        if (currentNode->data == goal) {
            goalNode = *currentNode;
            break;
        }
        let neighbors = neighborFn(currentNode->data);
        for (let neighbor : neighbors) {
            NodePtr neighborNode = emplace(
                searchState.nodeArena, 
                neighbor, 
                std::make_shared<Node>(neighbor));

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