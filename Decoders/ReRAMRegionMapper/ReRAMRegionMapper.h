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

#ifndef __RERAM_REGION_MAPPER_H__
#define __RERAM_REGION_MAPPER_H__

#include "src/AddressTranslator.h"
#include "src/Config.h"
#include "include/NVMAddress.h"
#include <map>
#include <cstdint>

namespace NVM {

/**
 * @class ReRAMRegionMapper
 * @brief Address translator implementing dynamic region-based mapping for ReRAM
 *
 * This decoder implements hardware-managed virtual-to-physical region mapping for
 * ReRAM wear leveling and latency optimization. It translates Virtual Row Addresses
 * (VRA) to Physical Row Addresses (PRA) using a per-bank Region Table.
 *
 * Address Translation Flow:
 *   1. Receive Physical Address (PA) from CPU/gem5
 *   2. Extract Virtual Row Address (VRA) = PA[32:17] (16 bits)
 *   3. Split VRA into VRN (10 bits) + Region Offset (6 bits)
 *   4. Lookup Region Table: VRN → PRN (per bank)
 *   5. Reconstruct PRA = (PRN << 6) | Region Offset
 *   6. Send PRA to memory controller
 *
 * Region Configuration:
 *   - Region Size: 64 rows (2^6)
 *   - Regions per Bank: 1024 (64K rows / 64)
 *   - Regions per Mat: 16 (1024 rows / 64)
 *   - Fast Regions: First N regions of each mat (configurable)
 */
class ReRAMRegionMapper : public AddressTranslator
{
  public:
    ReRAMRegionMapper();
    ~ReRAMRegionMapper();

    void SetConfig(Config *config, bool createChildren = true);

    /**
     * @brief Translate physical address to memory address fields with region mapping
     *
     * Overrides parent Translate() to perform VRN→PRN translation on row field.
     *
     * @param address Physical address from CPU
     * @param row Output: Physical Row Address (PRA) after translation
     * @param col Output: Column address
     * @param bank Output: Bank ID
     * @param rank Output: Rank ID
     * @param channel Output: Channel ID
     * @param subarray Output: Mat (subarray) index
     */
    virtual void Translate(uint64_t address, uint64_t *row, uint64_t *col,
                          uint64_t *bank, uint64_t *rank, uint64_t *channel,
                          uint64_t *subarray);

    using AddressTranslator::Translate;

    /**
     * @brief Swap two virtual regions' physical mappings
     *
     * Atomically swaps the VRN→PRN mappings for two virtual regions in the
     * specified bank. Updates both forward and inverse region tables.
     *
     * Example: If VRN_hot maps to PRN_slow and VRN_cold maps to PRN_fast,
     *          after swap: VRN_hot → PRN_fast, VRN_cold → PRN_slow
     *
     * @param bank Bank ID (0-7)
     * @param VRN_hot Virtual region number with high access score
     * @param VRN_cold Virtual region number with low access score
     */
    void SwapRegions(uint64_t bank, uint64_t VRN_hot, uint64_t VRN_cold);

    /**
     * @brief Determine if a physical region is fast or slow
     *
     * Fast regions are the first N regions in each mat (near wordline driver).
     * Slow regions are the remaining regions in each mat (far from driver).
     *
     * @param PRN Physical Region Number (0-1023)
     * @return true if PRN maps to a fast physical region, false otherwise
     */
    bool IsFastRegion(uint64_t PRN) const;

    /**
     * @brief Get Virtual Region Number from Physical Region Number
     *
     * Uses inverse region table to map PRN back to VRN for score tracking.
     * This is needed because memory controller sees PRA (containing PRN) but
     * must track scores by VRN.
     *
     * @param bank Bank ID
     * @param PRN Physical Region Number
     * @return VRN Virtual Region Number
     */
    uint64_t GetVRNFromPRN(uint64_t bank, uint64_t PRN) const;

    /**
     * @brief Get Physical Region Number for a given bank and VRN
     *
     * @param bank Bank ID
     * @param VRN Virtual Region Number
     * @return PRN Physical Region Number
     */
    uint64_t GetPRN(uint64_t bank, uint64_t VRN) const;

    void RegisterStats();
    void CalculateStats();

    void CreateCheckpoint(std::string dir);
    void RestoreCheckpoint(std::string dir);

  private:
    /**
     * Region Table: VRN → PRN mapping (per bank)
     * Key encoding: (bank << 10) | VRN
     *   - bank: 3 bits (0-7 banks)
     *   - VRN: 10 bits (0-1023 regions per bank)
     * Value: PRN (10 bits)
     * Total entries: 8 banks × 1024 regions = 8192 entries (~10 KB)
     */
    std::map<uint64_t, uint64_t> regionTable;

    /**
     * Inverse Region Table: PRN → VRN mapping (for score tracking)
     * Same structure as regionTable, but reverse lookup.
     * Updated atomically with regionTable during swaps.
     */
    std::map<uint64_t, uint64_t> inverseRegionTable;

    // Configuration parameters (from .config file)
    uint64_t regionSize;           // Rows per region (default: 64)
    uint64_t numRegionsPerBank;    // Total regions per bank (default: 1024)
    uint64_t numRegionsPerMat;     // Regions per mat (default: 16)
    uint64_t matHeight;            // Rows per mat (default: 1024)
    uint64_t numMats;              // Mats per bank (default: 64)
    uint64_t fastRegionsPerMat;    // Fast regions per mat (default: 4)
    uint64_t numBanks;             // Total banks (default: 8)
    uint64_t numRows;              // Rows per bank (default: 65536)

    // Bit manipulation constants (for performance)
    static const int VRN_SHIFT = 6;        // log2(64) for VRA→VRN extraction
    static const int MAT_SHIFT = 10;       // log2(1024) for PRA→Mat extraction
    static const uint64_t RO_MASK = 0x3F;  // 0b111111 for 6-bit Region Offset

    /**
     * @brief Initialize region tables with identity mapping
     *
     * Sets up initial state where VRN == PRN for all regions.
     * Called during SetConfig().
     */
    void InitializeRegionTable();

    /**
     * @brief Extract Virtual Region Number from Virtual Row Address
     * @param VRA Virtual Row Address (16 bits)
     * @return VRN (top 10 bits of VRA)
     */
    inline uint64_t GetVRN(uint64_t VRA) const {
        return VRA >> VRN_SHIFT;
    }

    /**
     * @brief Extract Region Offset from Virtual Row Address
     * @param VRA Virtual Row Address (16 bits)
     * @return Region Offset (bottom 6 bits of VRA, range 0-63)
     */
    inline uint64_t GetRegionOffset(uint64_t VRA) const {
        return VRA & RO_MASK;
    }

    /**
     * @brief Reconstruct Physical Row Address from PRN and offset
     * @param PRN Physical Region Number (10 bits)
     * @param RO Region Offset (6 bits)
     * @return PRA Physical Row Address (16 bits)
     */
    inline uint64_t GetPRA(uint64_t PRN, uint64_t RO) const {
        return (PRN << VRN_SHIFT) | RO;
    }

    /**
     * @brief Generate unique key for region table lookup
     * @param bank Bank ID (0-7)
     * @param regionNum Region number (VRN or PRN depending on table)
     * @return Unique key encoding both bank and region number
     */
    inline uint64_t MakeKey(uint64_t bank, uint64_t regionNum) const {
        return (bank << 10) | regionNum;
    }

    // Statistics counters
    ncounter_t regionSwaps;          // Total number of region swaps performed
    ncounter_t fastRegionAccesses;   // Accesses to fast physical regions
    ncounter_t slowRegionAccesses;   // Accesses to slow physical regions
    ncounter_t totalTranslations;    // Total address translations performed
};

}; // namespace NVM

#endif // __RERAM_REGION_MAPPER_H__
