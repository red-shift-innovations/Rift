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
 * File:   zip.hpp
 * Author: Nick Otero
 * Website: www.redshiftinnovations.tech
 */

#pragma once

#include <algorithm>
#include <rift/util/rustify.hpp>

namespace rift {
    
    /**
     * Combine two ranges to make a single range containing pairs of elements
     * from each input range.
     * 
     * \tparam T the type of range to be combined
     * \param range1 the first range to combine
     * \param range2 the second range to combine
     * \return a range containing pairs of elements from both input ranges
     */
    template <typename T>
    fn zip(T range1, T range2) {
        using ValueType = typename T::value_type;
        std::vector<std::pair<ValueType, ValueType>> result;
        let makePair = [](let first, let second) { 
            return std::make_pair(first, second);
        };
        let doZip = [&](let first, let second) {
            std::transform(
                begin(first), 
                end(first), 
                begin(second),
                std::back_inserter(result),
                makePair);
        };
        if (range2.size() >= range1.size()) {
            doZip(range1, range2);
        } else {
            doZip(range2, range1);           
        }
        return result;
    }
}