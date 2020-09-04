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
 * File:   index.hpp
 * Author: Nick Otero
 * Website: www.redshiftinnovations.tech
 */

#pragma once

namespace rift {
    struct Index {
        std::size_t index;
        
        explicit Index(size_t i) : index(i) {}
        
        bool operator==(Index const& other) const noexcept {
            return index == other.index;
        }
        
        std::weak_ordering operator<=>(size_t other) const noexcept {
            return index <=> other;
        }
        
        Index operator+(size_t other) const noexcept { 
            return Index(index + other); 
        }
        
        Index operator-(size_t other) const noexcept { 
            return Index(index - other); 
        }
    };
    
    std::ostream& operator<<(std::ostream& out, Index index) noexcept {
        out << index.index
;        return out;
    }
}