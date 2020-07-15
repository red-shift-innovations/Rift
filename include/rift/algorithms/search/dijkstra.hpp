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

#include <concepts>
#include <limits>
#include <optional>
#include <type_traits>
#include <vector>

#include <rift/algorithms/search/search_concepts.hpp>
#include <rift/util/rustify.hpp>

namespace rift {
    template <std::equality_comparable T>
    struct DijkstraState {
        std::vector<T> openSet;
    };
    
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
        T* parent;
        
        /** The accumulated cost to come to this node*/
        C costToCome = std::numeric_limits<C>::max();
        
        /** Flag indicating if this node is on the open set */
        bool isOpen = false;
        
        /** Flag indicating if this node has been removed from the open set */
        bool isClosed = false;
    };
    
    template <std::equality_comparable T, NeighborFn<T> N, CostFn<T> G>
    fn dijkstra(T start, T goal, N neighborFn, G costFn) 
        -> std::optional<std::vector<T>> {
        
        using Cost = std::invoke_result_t<G, T, T>;
        using Node = DijkstraNode<T, Cost>;
        std::vector<T> openSet, result;
        Node startNode(start), goalNode(goal);
        startNode.costToCome = 0;
        result.push_back(start);
        result.push_back(goal);
        return result;
    }
}

