/*
 * Copyright (c) 2012-2015 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "mem/ruby/common/WriteMask.hh"

#include <string>

#include "mem/ruby/system/RubySystem.hh"

namespace gem5
{

namespace ruby
{

WriteMask::WriteMask()
    : mSize(0), mMask(mSize, false), mAtomic(false)
{}

void
WriteMask::print(std::ostream& out) const
{
    assert(mSize > 0);
    std::string str(mSize,'0');
    for (int i = 0; i < mSize; i++) {
        str[i] = mMask[i] ? ('1') : ('0');
    }
    out << "dirty mask="
        << str
        << std::flush;
}

void
WriteMask::performAtomic(uint8_t * p,
        std::deque<uint8_t*>& log, bool isAtomicNoReturn) const
{
    assert(mSize > 0);
    int offset;
    uint8_t *block_update;
    // Here, operations occur in FIFO order from the mAtomicOp
    // vector. This is done to match the ordering of packets
    // that was seen when the initial coalesced request was created.
    for (int i = 0; i < mAtomicOp.size(); i++) {
        if (!isAtomicNoReturn) {
            // Save the old value of the data block in case a
            // return value is needed
            block_update = new uint8_t[mSize];
            std::memcpy(block_update, p, mSize);
            log.push_back(block_update);
        }
        // Perform the atomic operation
        offset = mAtomicOp[i].first;
        AtomicOpFunctor *fnctr = mAtomicOp[i].second;
        (*fnctr)(&p[offset]);
    }
}

} // namespace ruby
} // namespace gem5
