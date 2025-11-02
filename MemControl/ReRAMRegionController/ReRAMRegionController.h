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

#ifndef __RERAM_REGION_CONTROLLER_H__
#define __RERAM_REGION_CONTROLLER_H__

#include "MemControl/FRFCFS/FRFCFS.h"
#include "Decoders/ReRAMRegionMapper/ReRAMRegionMapper.h"
#include "include/NVMainRequest.h"
#include <map>
#include <vector>
#include <cstdint>

namespace NVM {

/**
 * @class ReRAMRegionController
 * @brief Memory controller with region-based scoring and migration for ReRAM
 *
 * Extends FRFCFS controller to add:
 * 1. Per-region access score tracking (write and read counts)
 * 2. Epoch-based migration decisions (triggered every N cycles)
 * 3. Dynamic region swapping (hot virtual regions → fast physical regions)
 *
 * Scoring Algorithm:
 *   S(VRN) = α × WS(VRN) + β × RS(VRN)
 *   Where:
 *     - WS = Write Score (write count for virtual region)
 *     - RS = Read Score (read count for virtual region)
 *     - α = write weight (default 0.5)
 *     - β = read weight (default 0.5)
 *
 * Migration Policy:
 *   - Every epoch (e.g., 1M cycles), calculate scores for all regions
 *   - Find VRN_max (highest score) and VRN_min (lowest score)
 *   - If (S_max - S_min) > threshold AND VRN_max is in slow region:
 *       Swap VRN_max with VRN mapped to fast region with lowest score
 *   - Halve all scores to emphasize recent behavior
 *
 * Critical Design Note:
 *   The controller receives requests with PRA (Physical Row Address) because
 *   the AddressTranslator has already performed VRA→PRA translation. To track
 *   scores by VRN, we must use the ReRAMRegionMapper's inverse table to map
 *   PRN back to VRN.
 */
class ReRAMRegionController : public FRFCFS
{
  public:
    ReRAMRegionController();
    ~ReRAMRegionController();

    void SetConfig(Config *conf, bool createChildren = true);

    /**
     * @brief Issue command and update region scores
     *
     * Overrides FRFCFS::IssueCommand to track read/write access counts
     * per virtual region for scoring algorithm.
     */
    bool IssueCommand(NVMainRequest *req);

    /**
     * @brief Cycle function with epoch-based migration
     *
     * Extends FRFCFS::Cycle to check for epoch boundaries and trigger
     * migration decisions when appropriate.
     */
    void Cycle(ncycle_t steps);

    void RegisterStats();
    void CalculateStats();

    /**
     * @brief Set reference to the region mapper
     *
     * Must be called after construction to enable region swapping.
     * Typically called during initialization by NVMain.
     */
    void SetRegionMapper(ReRAMRegionMapper *mapper);

  private:
    // Reference to address translator for region swapping
    ReRAMRegionMapper *regionMapper;

    // Configuration parameters (from .config file)
    double alpha;                   // Write score weight (default: 0.5)
    double beta;                    // Read score weight (default: 0.5)
    ncycle_t epochLength;           // Migration decision interval (default: 1M cycles)
    double migrationThreshold;      // Min (S_max - S_min) to trigger swap (default: 100)
    uint64_t numBanks;              // Number of banks
    uint64_t numRegionsPerBank;     // Regions per bank (default: 1024)
    uint64_t numRegionsPerMat;      // Regions per mat (default: 16)
    uint64_t fastRegionsPerMat;     // Fast regions per mat (default: 4)

    // Epoch tracking
    ncycle_t currentCycle;
    ncycle_t lastEpochCycle;

    /**
     * Per-bank, per-VRN score tracking
     * Key encoding: (bank << 10) | VRN
     *   - bank: 3 bits (0-7 banks)
     *   - VRN: 10 bits (0-1023 regions per bank)
     */
    std::map<uint64_t, uint64_t> writeScores;   // WS: write count per VRN
    std::map<uint64_t, uint64_t> readScores;    // RS: read count per VRN
    std::map<uint64_t, double> regionScores;    // S = α*WS + β*RS

    /**
     * @brief Update scores when processing memory request
     *
     * Extracts VRN from request address (using inverse lookup) and
     * increments appropriate score counter.
     *
     * @param req Memory request being processed
     */
    void UpdateRegionScores(NVMainRequest *req);

    /**
     * @brief Calculate combined scores for all regions in a bank
     *
     * Computes S(VRN) = α*WS + β*RS for all virtual regions in the bank.
     *
     * @param bank Bank ID
     */
    void CalculateRegionScores(uint64_t bank);

    /**
     * @brief Check if migration threshold is met
     *
     * Finds VRN with max and min scores. Returns true if score difference
     * exceeds threshold AND max-score VRN is currently mapped to slow region.
     *
     * @param bank Bank ID
     * @param VRN_max Output: VRN with highest score
     * @param VRN_min Output: VRN with lowest score
     * @return true if migration should be performed
     */
    bool CheckMigrationThreshold(uint64_t bank, uint64_t &VRN_max, uint64_t &VRN_min);

    /**
     * @brief Perform migration for a bank
     *
     * Algorithm:
     *   1. Calculate scores for all regions
     *   2. Find VRN_hot (max score) and check if in slow physical region
     *   3. Find VRN_cold in fast physical region with lowest score
     *   4. Swap VRN_hot ↔ VRN_cold mappings
     *   5. Halve all scores to emphasize recent accesses
     *
     * @param bank Bank ID
     */
    void Migration(uint64_t bank);

    /**
     * @brief Halve all scores to emphasize recent behavior
     *
     * Called after each migration to decay old access counts.
     * Prevents score overflow and adapts to changing workloads.
     */
    void HalveScores();

    /**
     * @brief Find VRN in fast region with lowest score
     *
     * Used to identify candidate for swapping out when migrating
     * a hot region into fast physical region.
     *
     * @param bank Bank ID
     * @return VRN of cold region currently in fast physical region
     */
    uint64_t FindColdFastRegion(uint64_t bank);

    /**
     * @brief Generate unique key for score tracking
     * @param bank Bank ID
     * @param VRN Virtual Region Number
     * @return Unique key encoding both bank and VRN
     */
    inline uint64_t MakeKey(uint64_t bank, uint64_t VRN) const {
        return (bank << 10) | VRN;
    }

    // Statistics counters
    ncounter_t totalMigrations;        // Total migrations performed
    ncounter_t totalEpochs;            // Total epochs completed
    ncounter_t migrationsPerBank[8];   // Per-bank migration counts
    ncounter_t hotAccessesToFast;      // Hot regions accessing fast areas (success metric)
    ncounter_t hotAccessesToSlow;      // Hot regions accessing slow areas (needs migration)

    double avgScoreDifference;         // Average score difference at migration
    double maxScoreDifference;         // Maximum score difference observed
};

}; // namespace NVM

#endif // __RERAM_REGION_CONTROLLER_H__
