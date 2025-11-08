/*******************************************************************************
* Copyright (c) 2012-2014, The Microsystems Design Labratory (MDL)
* Department of Computer Science and Engineering, The Pennsylvania State University
* All rights reserved.
*
* This source code is part of NVMain - A cycle accurate timing, bit accurate
* energy simulator for both volatile (e.g., DRAM) and non-volatile memory
* (e.g., PCRAM). The source code is free and you can redistribute and/or
* modify it by providing that the following conditions are met:
*
*  1) Redistributions of source code must retain the above copyright notice,
*     this list of conditions and the following disclaimer.
*
*  2) Redistributions in binary form must reproduce the above copyright notice,
*     this list of conditions and the following disclaimer in the documentation
*     and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Author: Dynamic ReRAM Region Mapping Implementation
*******************************************************************************/

#include "Banks/ReRAMBank/ReRAMBank.h"
#include "src/EventQueue.h"
#include <iostream>
#include <cmath>

using namespace NVM;

ReRAMBank::ReRAMBank()
    : DDR3Bank()
{
    // Default latency values (will be overridden by config)
    fastRegionLatency = 50;   // 50ns for fast regions
    slowRegionLatency = 120;  // 120ns for slow regions
    fastRegionsPerMat = 4;    // First 4 of 16 regions are fast
    numRegionsPerMat = 16;
    clkFreqMHz = 667.0;       // DDR3-1333 I/O frequency

    // Initialize statistics
    fastRegionWrites = 0;
    slowRegionWrites = 0;
    fastRegionReads = 0;
    slowRegionReads = 0;
    totalFastWriteLatency = 0.0;
    totalSlowWriteLatency = 0.0;
}

ReRAMBank::~ReRAMBank()
{
}

void ReRAMBank::SetConfig(Config *c, bool createChildren)
{
    // Call parent SetConfig first
    DDR3Bank::SetConfig(c, createChildren);

    // Read ReRAM-specific configuration
    if (c->KeyExists("FastRegionLatency"))
        fastRegionLatency = c->GetValue("FastRegionLatency");

    if (c->KeyExists("SlowRegionLatency"))
        slowRegionLatency = c->GetValue("SlowRegionLatency");

    if (c->KeyExists("FastRegionsPerMat"))
        fastRegionsPerMat = c->GetValue("FastRegionsPerMat");

    if (c->KeyExists("MATHeight")) {
        uint64_t matHeight = c->GetValue("MATHeight");
        uint64_t regionSize = 64;
        if (c->KeyExists("RegionSize"))
            regionSize = c->GetValue("RegionSize");
        numRegionsPerMat = matHeight / regionSize;
    }

    // Get clock frequency for latency conversion
    if (c->KeyExists("CLK"))
        clkFreqMHz = c->GetValue("CLK");

    // Print configuration
    std::cout << "ReRAMBank Configuration:" << std::endl;
    std::cout << "  Fast region latency: " << fastRegionLatency << " ns" << std::endl;
    std::cout << "  Slow region latency: " << slowRegionLatency << " ns" << std::endl;
    std::cout << "  Fast regions per mat: " << fastRegionsPerMat
              << " / " << numRegionsPerMat << std::endl;
    std::cout << "  Clock frequency: " << clkFreqMHz << " MHz" << std::endl;
    std::cout << "  Fast region latency (cycles): " << NanoToCycles(fastRegionLatency) << std::endl;
    std::cout << "  Slow region latency (cycles): " << NanoToCycles(slowRegionLatency) << std::endl;
}

bool ReRAMBank::IsFastRegion(uint64_t PRA) const
{
    // Extract region index within mat from PRA
    // PRA[9:6] = region within mat (0-15)
    uint64_t regionInMat = (PRA >> 6) & 0xF;

    // First fastRegionsPerMat regions in each mat are fast
    return (regionInMat < fastRegionsPerMat);
}

ncycle_t ReRAMBank::NanoToCycles(double ns) const
{
    // Convert nanoseconds to clock cycles
    // cycles = (ns × MHz) / 1000
    // Example: 50ns @ 667MHz = (50 × 667) / 1000 = 33.35 ≈ 34 cycles
    double cycles = (ns * clkFreqMHz) / 1000.0;
    return static_cast<ncycle_t>(std::ceil(cycles));
}

bool ReRAMBank::IssueCommand(NVMainRequest *req)
{
    // Check operation type and delegate to appropriate handler
    OpType op = req->type;

    if (op == WRITE || op == WRITE_PRECHARGE) {
        return Write(req);
    } else if (op == READ || op == READ_PRECHARGE) {
        return Read(req);
    } else {
        // For other operations (ACTIVATE, PRECHARGE, etc.), use parent handler
        return DDR3Bank::IssueCommand(req);
    }
}

bool ReRAMBank::Write(NVMainRequest *request)
{
    // Extract PRA from request
    uint64_t PRA = request->address.GetRow();

    // Determine if this is a fast or slow region
    bool isFast = IsFastRegion(PRA);

    // Update statistics
    if (isFast) {
        fastRegionWrites++;
        totalFastWriteLatency += fastRegionLatency;
    } else {
        slowRegionWrites++;
        totalSlowWriteLatency += slowRegionLatency;
    }

    // Call parent Write method
    // Note: The actual latency is handled in NextIssuable() method
    // which the memory controller uses for scheduling
    bool success = DDR3Bank::Write(request);

    #ifdef DEBUG_RERAM_BANK
    if (success) {
        std::cout << "ReRAMBank Write: PRA=" << PRA
                  << ", region=" << ((PRA >> 6) & 0xF)
                  << ", fast=" << isFast
                  << ", latency=" << (isFast ? fastRegionLatency : slowRegionLatency) << " ns"
                  << std::endl;
    }
    #endif

    return success;
}

bool ReRAMBank::Read(NVMainRequest *request)
{
    // Extract PRA from request
    uint64_t PRA = request->address.GetRow();

    // Determine if this is a fast or slow region (for statistics)
    bool isFast = IsFastRegion(PRA);

    if (isFast) {
        fastRegionReads++;
    } else {
        slowRegionReads++;
    }

    // For reads, use uniform latency (parent implementation)
    // ReRAM read latency is typically less sensitive to distance
    // If variable read latency is needed, implement similar to Write
    return DDR3Bank::Read(request);
}

ncycle_t ReRAMBank::NextIssuable(NVMainRequest *request)
{
    // Get base next issuable time from parent
    ncycle_t baseIssuable = DDR3Bank::NextIssuable(request);

    // For write operations, add variable latency based on region
    // Note: This is a simplified approach. In a more accurate model,
    // we would modify the timing parameters (tWR, etc.) in SetConfig
    // based on the region being accessed.
    OpType op = request->type;
    if (op == WRITE || op == WRITE_PRECHARGE) {
        uint64_t PRA = request->address.GetRow();
        bool isFast = IsFastRegion(PRA);

        // Add additional cycles for slow regions
        // Fast regions use baseline timing, slow regions add delay
        if (!isFast) {
            ncycle_t extraLatency = NanoToCycles(slowRegionLatency - fastRegionLatency);
            return baseIssuable + extraLatency;
        }
    }

    return baseIssuable;
}

void ReRAMBank::RegisterStats()
{
    DDR3Bank::RegisterStats();

    AddStat(fastRegionWrites);
    AddStat(slowRegionWrites);
    AddStat(fastRegionReads);
    AddStat(slowRegionReads);
}

void ReRAMBank::CalculateStats()
{
    DDR3Bank::CalculateStats();

    std::cout << "\nReRAMBank Statistics:" << std::endl;

    // Write statistics
    uint64_t totalWrites = fastRegionWrites + slowRegionWrites;
    std::cout << "  Write Operations:" << std::endl;
    std::cout << "    Fast region writes: " << fastRegionWrites;
    if (totalWrites > 0) {
        std::cout << " (" << (100.0 * fastRegionWrites / totalWrites) << "%)";
    }
    std::cout << std::endl;

    std::cout << "    Slow region writes: " << slowRegionWrites;
    if (totalWrites > 0) {
        std::cout << " (" << (100.0 * slowRegionWrites / totalWrites) << "%)";
    }
    std::cout << std::endl;

    if (fastRegionWrites > 0) {
        std::cout << "    Avg fast write latency: "
                  << (totalFastWriteLatency / fastRegionWrites) << " ns" << std::endl;
    }
    if (slowRegionWrites > 0) {
        std::cout << "    Avg slow write latency: "
                  << (totalSlowWriteLatency / slowRegionWrites) << " ns" << std::endl;
    }

    // Overall average write latency
    if (totalWrites > 0) {
        double avgWriteLatency = (totalFastWriteLatency + totalSlowWriteLatency) / totalWrites;
        std::cout << "    Overall avg write latency: " << avgWriteLatency << " ns" << std::endl;

        // Calculate latency reduction compared to all-slow baseline
        double baselineLatency = slowRegionLatency * totalWrites;
        double actualLatency = totalFastWriteLatency + totalSlowWriteLatency;
        double latencyReduction = ((baselineLatency - actualLatency) / baselineLatency) * 100.0;
        std::cout << "    Latency reduction vs all-slow: " << latencyReduction << "%" << std::endl;
    }

    // Read statistics
    uint64_t totalReads = fastRegionReads + slowRegionReads;
    std::cout << "  Read Operations:" << std::endl;
    std::cout << "    Fast region reads: " << fastRegionReads;
    if (totalReads > 0) {
        std::cout << " (" << (100.0 * fastRegionReads / totalReads) << "%)";
    }
    std::cout << std::endl;

    std::cout << "    Slow region reads: " << slowRegionReads;
    if (totalReads > 0) {
        std::cout << " (" << (100.0 * slowRegionReads / totalReads) << "%)";
    }
    std::cout << std::endl;
}
