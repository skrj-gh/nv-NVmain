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

#ifndef __RERAM_BANK_H__
#define __RERAM_BANK_H__

#include "Banks/DDR3Bank/DDR3Bank.h"
#include "include/NVMainRequest.h"
#include <cstdint>

namespace NVM {

/**
 * @class ReRAMBank
 * @brief Bank module with variable latency for ReRAM fast/slow regions
 *
 * Extends DDR3Bank to support different access latencies based on physical
 * region location within the ReRAM crossbar array.
 *
 * Key Concept:
 *   ReRAM crossbar arrays exhibit distance-dependent latency. Rows near the
 *   wordline driver (near-end rows) have lower access latency than rows far
 *   from the driver (far-end rows).
 *
 * Region Classification:
 *   - Fast Physical Region (FPR): First N regions of each mat (near WL driver)
 *   - Slow Physical Region (SPR): Remaining regions (far from WL driver)
 *
 * Latency Model:
 *   - Fast region latency: ~50ns (configurable via FastRegionLatency)
 *   - Slow region latency: ~120ns (configurable via SlowRegionLatency)
 *   - Read latency: Uniform (distance-independent for resistive sensing)
 *   - Write latency: Distance-dependent (affects SET/RESET operations)
 *
 * Physical Address Decomposition:
 *   PRA (Physical Row Address) = [Mat Index(6) | Region in Mat(4) | Row in Region(6)]
 *   - Mat Index = PRA[15:10] (0-63 mats per bank)
 *   - Region in Mat = PRA[9:6] (0-15 regions per mat)
 *   - Row in Region = PRA[5:0] (0-63 rows per region)
 *
 * Fast Region Determination:
 *   isFast = (PRA[9:6] < FastRegionsPerMat)
 *
 * NOTE: This implementation focuses on write latency variation. Reads could
 * also be made variable if needed for more detailed modeling.
 */
class ReRAMBank : public DDR3Bank
{
  public:
    ReRAMBank();
    ~ReRAMBank();

    void SetConfig(Config *c, bool createChildren = true);

    /**
     * @brief Issue command with region-aware latency
     *
     * Overrides DDR3Bank::IssueCommand to apply different latencies
     * based on whether the physical row is in a fast or slow region.
     */
    bool IssueCommand(NVMainRequest *req);

    /**
     * @brief Calculate next issuable time with variable latency
     *
     * Determines when the next command to this bank can be issued,
     * considering the variable latency of fast vs slow regions.
     */
    ncycle_t NextIssuable(NVMainRequest *request);

    void RegisterStats();
    void CalculateStats();

  protected:
    /**
     * @brief Write operation with variable latency
     *
     * Overrides DDR3Bank::Write to apply fast/slow region latency.
     */
    bool Write(NVMainRequest *request);

    /**
     * @brief Read operation (uniform latency for now)
     *
     * Can be extended to support variable read latency if needed.
     */
    bool Read(NVMainRequest *request);

  private:
    // Configuration parameters
    ncycle_t fastRegionLatency;     // Write latency for fast regions (ns)
    ncycle_t slowRegionLatency;     // Write latency for slow regions (ns)
    uint64_t fastRegionsPerMat;     // Number of fast regions per mat
    uint64_t numRegionsPerMat;      // Total regions per mat (default: 16)
    double clkFreqMHz;              // Clock frequency in MHz

    /**
     * @brief Determine if a physical row is in a fast region
     *
     * Fast regions are the first N regions in each mat.
     *
     * @param PRA Physical Row Address
     * @return true if PRA maps to fast region, false otherwise
     */
    bool IsFastRegion(uint64_t PRA) const;

    /**
     * @brief Convert nanoseconds to clock cycles
     *
     * @param ns Time in nanoseconds
     * @return Time in clock cycles
     */
    ncycle_t NanoToCycles(double ns) const;

    // Statistics
    ncounter_t fastRegionWrites;    // Write count to fast regions
    ncounter_t slowRegionWrites;    // Write count to slow regions
    ncounter_t fastRegionReads;     // Read count from fast regions
    ncounter_t slowRegionReads;     // Read count from slow regions

    double totalFastWriteLatency;   // Accumulated fast write latency (ns)
    double totalSlowWriteLatency;   // Accumulated slow write latency (ns)
};

}; // namespace NVM

#endif // __RERAM_BANK_H__
