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

#include "MemControl/ReRAMRegionController/ReRAMRegionController.h"
#include <iostream>
#include <algorithm>
#include <limits>
#include <cassert>

using namespace NVM;

ReRAMRegionController::ReRAMRegionController()
    : FRFCFS()
{
    regionMapper = NULL;

    // Default configuration values
    alpha = 0.5;
    beta = 0.5;
    epochLength = 1000000;  // 1M cycles
    migrationThreshold = 100.0;
    numBanks = 8;
    numRegionsPerBank = 1024;
    numRegionsPerMat = 16;
    fastRegionsPerMat = 4;

    // Initialize cycle tracking
    currentCycle = 0;
    lastEpochCycle = 0;

    // Initialize statistics
    totalMigrations = 0;
    totalEpochs = 0;
    for (int i = 0; i < 8; i++) {
        migrationsPerBank[i] = 0;
    }
    hotAccessesToFast = 0;
    hotAccessesToSlow = 0;
    avgScoreDifference = 0.0;
    maxScoreDifference = 0.0;
}

ReRAMRegionController::~ReRAMRegionController()
{
    writeScores.clear();
    readScores.clear();
    regionScores.clear();
}

void ReRAMRegionController::SetConfig(Config *conf, bool createChildren)
{
    // Call parent SetConfig first
    FRFCFS::SetConfig(conf, createChildren);

    // Read configuration parameters
    if (conf->KeyExists("BANKS"))
        numBanks = conf->GetValue("BANKS");

    if (conf->KeyExists("ROWS")) {
        uint64_t numRows = conf->GetValue("ROWS");
        uint64_t regionSize = 64;
        if (conf->KeyExists("RegionSize"))
            regionSize = conf->GetValue("RegionSize");
        numRegionsPerBank = numRows / regionSize;
    }

    if (conf->KeyExists("MATHeight")) {
        uint64_t matHeight = conf->GetValue("MATHeight");
        uint64_t regionSize = 64;
        if (conf->KeyExists("RegionSize"))
            regionSize = conf->GetValue("RegionSize");
        numRegionsPerMat = matHeight / regionSize;
    }

    if (conf->KeyExists("FastRegionsPerMat"))
        fastRegionsPerMat = conf->GetValue("FastRegionsPerMat");

    if (conf->KeyExists("Alpha"))
        alpha = conf->GetValue("Alpha");

    if (conf->KeyExists("Beta"))
        beta = conf->GetValue("Beta");

    if (conf->KeyExists("EpochLength"))
        epochLength = conf->GetValue("EpochLength");

    if (conf->KeyExists("MigrationThreshold"))
        migrationThreshold = conf->GetValue("MigrationThreshold");

    // Validation
    assert(alpha + beta > 0.0 && "Alpha + Beta must be positive");
    assert(epochLength > 0 && "EpochLength must be positive");
    assert(migrationThreshold >= 0.0 && "MigrationThreshold must be non-negative");

    // Print configuration
    std::cout << "ReRAMRegionController Configuration:" << std::endl;
    std::cout << "  Alpha (write weight): " << alpha << std::endl;
    std::cout << "  Beta (read weight): " << beta << std::endl;
    std::cout << "  Epoch length: " << epochLength << " cycles" << std::endl;
    std::cout << "  Migration threshold: " << migrationThreshold << std::endl;
    std::cout << "  Banks: " << numBanks << std::endl;
    std::cout << "  Regions per bank: " << numRegionsPerBank << std::endl;
    std::cout << "  Fast regions per mat: " << fastRegionsPerMat
              << " / " << numRegionsPerMat << std::endl;
}

void ReRAMRegionController::SetRegionMapper(ReRAMRegionMapper *mapper)
{
    assert(mapper != NULL && "Region mapper cannot be NULL");
    regionMapper = mapper;
    std::cout << "ReRAMRegionController: Region mapper reference set" << std::endl;
}

bool ReRAMRegionController::IssueCommand(NVMainRequest *req)
{
    // Call parent IssueCommand first
    bool issued = FRFCFS::IssueCommand(req);

    // If command was successfully issued, update region scores
    if (issued) {
        UpdateRegionScores(req);
    }

    return issued;
}

void ReRAMRegionController::UpdateRegionScores(NVMainRequest *req)
{
    // Skip if region mapper not set
    if (regionMapper == NULL) {
        return;
    }

    // Extract address components
    NVMAddress &addr = req->address;
    uint64_t bank = addr.GetBank();
    uint64_t PRA = addr.GetRow();  // This is Physical Row Address (already translated)

    // Extract PRN from PRA
    uint64_t PRN = PRA >> 6;  // Top 10 bits of PRA

    // Use inverse region table to get original VRN
    uint64_t VRN = regionMapper->GetVRNFromPRN(bank, PRN);

    // Generate key for score tracking
    uint64_t key = MakeKey(bank, VRN);

    // Update appropriate score based on operation type
    OpType op = req->type;
    if (op == WRITE || op == WRITE_PRECHARGE) {
        writeScores[key]++;
    } else if (op == READ || op == READ_PRECHARGE) {
        readScores[key]++;
    }

    // Track hot region effectiveness
    // Calculate combined score for this VRN
    double score = alpha * writeScores[key] + beta * readScores[key];

    // Check if this is a hot region (score > average)
    // and whether it's accessing fast or slow physical region
    bool isFast = regionMapper->IsFastRegion(PRN);
    if (score > migrationThreshold) {  // Simple heuristic for "hot"
        if (isFast) {
            hotAccessesToFast++;
        } else {
            hotAccessesToSlow++;
        }
    }
}

void ReRAMRegionController::Cycle(ncycle_t steps)
{
    // Update cycle counter
    currentCycle += steps;

    // Call parent Cycle for normal FRFCFS scheduling
    FRFCFS::Cycle(steps);

    // Check if epoch has ended
    if (currentCycle - lastEpochCycle >= epochLength) {
        totalEpochs++;

        // Trigger migration for each bank
        for (uint64_t bank = 0; bank < numBanks; bank++) {
            Migration(bank);
        }

        // Update epoch timestamp
        lastEpochCycle = currentCycle;

        #ifdef DEBUG_REGION_CONTROLLER
        std::cout << "ReRAMRegionController: Epoch " << totalEpochs
                  << " completed at cycle " << currentCycle << std::endl;
        std::cout << "  Total migrations: " << totalMigrations << std::endl;
        std::cout << "  Hot→Fast: " << hotAccessesToFast
                  << ", Hot→Slow: " << hotAccessesToSlow << std::endl;
        #endif
    }
}

void ReRAMRegionController::CalculateRegionScores(uint64_t bank)
{
    // Calculate combined score S = α*WS + β*RS for all regions in bank
    for (uint64_t VRN = 0; VRN < numRegionsPerBank; VRN++) {
        uint64_t key = MakeKey(bank, VRN);

        uint64_t WS = writeScores[key];  // Defaults to 0 if not in map
        uint64_t RS = readScores[key];   // Defaults to 0 if not in map

        double score = alpha * WS + beta * RS;
        regionScores[key] = score;
    }
}

bool ReRAMRegionController::CheckMigrationThreshold(uint64_t bank,
                                                     uint64_t &VRN_max,
                                                     uint64_t &VRN_min)
{
    // Find VRN with maximum and minimum scores
    double maxScore = -1.0;
    double minScore = std::numeric_limits<double>::max();

    VRN_max = 0;
    VRN_min = 0;

    for (uint64_t VRN = 0; VRN < numRegionsPerBank; VRN++) {
        uint64_t key = MakeKey(bank, VRN);
        double score = regionScores[key];

        if (score > maxScore) {
            maxScore = score;
            VRN_max = VRN;
        }
        if (score < minScore) {
            minScore = score;
            VRN_min = VRN;
        }
    }

    // Calculate score difference
    double scoreDiff = maxScore - minScore;

    // Update statistics
    if (scoreDiff > maxScoreDifference) {
        maxScoreDifference = scoreDiff;
    }
    avgScoreDifference = (avgScoreDifference * totalMigrations + scoreDiff) /
                         (totalMigrations + 1);

    // Check threshold
    if (scoreDiff < migrationThreshold) {
        return false;  // Not enough difference to warrant migration
    }

    // Check if hot region is already in fast physical region
    if (regionMapper != NULL) {
        uint64_t PRN_max = regionMapper->GetPRN(bank, VRN_max);
        if (regionMapper->IsFastRegion(PRN_max)) {
            return false;  // Already optimally placed
        }
    }

    return true;  // Migration should be performed
}

void ReRAMRegionController::Migration(uint64_t bank)
{
    // Skip if region mapper not set
    if (regionMapper == NULL) {
        std::cerr << "WARNING: Migration called but region mapper not set" << std::endl;
        return;
    }

    // Step 1: Calculate scores for all regions
    CalculateRegionScores(bank);

    // Step 2: Check if migration threshold is met
    uint64_t VRN_hot, VRN_cold_initial;
    if (!CheckMigrationThreshold(bank, VRN_hot, VRN_cold_initial)) {
        return;  // No migration needed
    }

    // Step 3: Find cold region currently in fast physical region
    uint64_t VRN_cold = FindColdFastRegion(bank);

    if (VRN_cold == VRN_hot) {
        // Edge case: hot region is the only one being tracked
        return;
    }

    // Step 4: Perform swap
    regionMapper->SwapRegions(bank, VRN_hot, VRN_cold);

    // Step 5: Update statistics
    totalMigrations++;
    migrationsPerBank[bank]++;

    // Step 6: Halve all scores to emphasize recent behavior
    HalveScores();

    #ifdef DEBUG_REGION_CONTROLLER
    uint64_t PRN_hot_new = regionMapper->GetPRN(bank, VRN_hot);
    uint64_t PRN_cold_new = regionMapper->GetPRN(bank, VRN_cold);
    std::cout << "Migration in bank " << bank << ":" << std::endl;
    std::cout << "  Swapped VRN " << VRN_hot << " (score=" << regionScores[MakeKey(bank, VRN_hot)]
              << ", now PRN=" << PRN_hot_new << ", fast=" << regionMapper->IsFastRegion(PRN_hot_new)
              << ")" << std::endl;
    std::cout << "     with VRN " << VRN_cold << " (score=" << regionScores[MakeKey(bank, VRN_cold)]
              << ", now PRN=" << PRN_cold_new << ", fast=" << regionMapper->IsFastRegion(PRN_cold_new)
              << ")" << std::endl;
    #endif
}

uint64_t ReRAMRegionController::FindColdFastRegion(uint64_t bank)
{
    double minScore = std::numeric_limits<double>::max();
    uint64_t coldestVRN = 0;
    bool foundFast = false;

    // Search for VRN mapped to fast region with lowest score
    for (uint64_t VRN = 0; VRN < numRegionsPerBank; VRN++) {
        // Get PRN for this VRN
        uint64_t PRN = regionMapper->GetPRN(bank, VRN);

        // Check if it's in a fast physical region
        if (regionMapper->IsFastRegion(PRN)) {
            foundFast = true;
            uint64_t key = MakeKey(bank, VRN);
            double score = regionScores[key];

            if (score < minScore) {
                minScore = score;
                coldestVRN = VRN;
            }
        }
    }

    if (!foundFast) {
        std::cerr << "WARNING: No fast region found in bank " << bank << std::endl;
    }

    return coldestVRN;
}

void ReRAMRegionController::HalveScores()
{
    // Halve all write scores
    for (auto &entry : writeScores) {
        entry.second = entry.second / 2;
    }

    // Halve all read scores
    for (auto &entry : readScores) {
        entry.second = entry.second / 2;
    }

    // Clear region scores (will be recalculated next epoch)
    regionScores.clear();
}

void ReRAMRegionController::RegisterStats()
{
    FRFCFS::RegisterStats();

    AddStat(totalMigrations);
    AddStat(totalEpochs);
    for (int i = 0; i < 8; i++) {
        AddStat(migrationsPerBank[i]);
    }
    AddStat(hotAccessesToFast);
    AddStat(hotAccessesToSlow);
}

void ReRAMRegionController::CalculateStats()
{
    FRFCFS::CalculateStats();

    std::cout << "\nReRAMRegionController Statistics:" << std::endl;
    std::cout << "  Total epochs: " << totalEpochs << std::endl;
    std::cout << "  Total migrations: " << totalMigrations << std::endl;

    if (totalEpochs > 0) {
        std::cout << "  Migrations per epoch: "
                  << (double)totalMigrations / totalEpochs << std::endl;
    }

    std::cout << "  Per-bank migrations:" << std::endl;
    for (uint64_t i = 0; i < numBanks; i++) {
        std::cout << "    Bank " << i << ": " << migrationsPerBank[i] << std::endl;
    }

    std::cout << "  Hot region effectiveness:" << std::endl;
    std::cout << "    Accesses to fast regions: " << hotAccessesToFast << std::endl;
    std::cout << "    Accesses to slow regions: " << hotAccessesToSlow << std::endl;

    uint64_t totalHotAccesses = hotAccessesToFast + hotAccessesToSlow;
    if (totalHotAccesses > 0) {
        std::cout << "    Fast region hit rate: "
                  << (100.0 * hotAccessesToFast / totalHotAccesses) << "%" << std::endl;
    }

    std::cout << "  Score difference statistics:" << std::endl;
    std::cout << "    Average: " << avgScoreDifference << std::endl;
    std::cout << "    Maximum: " << maxScoreDifference << std::endl;
}
