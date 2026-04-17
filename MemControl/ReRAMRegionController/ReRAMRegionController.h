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

struct MigrationCandidate
{
    bool valid;
    bool interbank;
    VirtualRegionLoc sourceVirt;
    VirtualRegionLoc victimVirt;
    PhysicalRegionLoc sourcePhys;
    PhysicalRegionLoc victimPhys;
    double hotScore;
    double coldScore;
    double rawBenefit;
    double migrationCost;
    double weightedBenefit;
};

class ReRAMRegionController : public FRFCFS
{
  public:
    ReRAMRegionController();
    ~ReRAMRegionController();

    void SetConfig(Config *conf, bool createChildren = true);
    bool IssueCommand(NVMainRequest *req);
    void Cycle(ncycle_t steps);

    void RegisterStats();
    void CalculateStats();

    void SetRegionMapper(ReRAMRegionMapper *mapper);

  private:
    ReRAMRegionMapper *regionMapper;

    double alpha;
    double beta;
    ncycle_t epochLength;
    double migrationThreshold;
    bool enableInterBankMigration;
    double interBankMigrationThreshold;
    double interBankMigrationCost;
    double interBankScoreWeight;
    double intraBankScoreWeight;
    uint64_t maxInterBankMigrationsPerEpoch;
    uint64_t maxTotalMigrationsPerEpoch;
    uint64_t interBankCooldownEpochs;
    bool preferIntraBankOnTie;
    bool allowCrossRankMigration;
    bool allowCrossChannelMigration;

    uint64_t numChannels;
    uint64_t numRanks;
    uint64_t numBanks;
    uint64_t numRegionsPerBank;
    uint64_t numRegionsPerMat;
    uint64_t fastRegionsPerMat;

    ncycle_t currentCycle;
    ncycle_t lastEpochCycle;

    std::map<uint64_t, uint64_t> writeScores;
    std::map<uint64_t, uint64_t> readScores;
    std::map<uint64_t, double> regionScores;
    std::map<uint64_t, uint64_t> lastMigrationEpoch;

    void UpdateRegionScores(NVMainRequest *req);
    void CalculateRegionScores(uint64_t channel, uint64_t rank, uint64_t bank);
    void Migration(uint64_t channel, uint64_t rank, uint64_t bank,
                   uint64_t& totalUsed, uint64_t& interbankUsed);
    void HalveScores();

    bool IsCoolingDown(const VirtualRegionLoc& loc) const;
    bool FindHotSlowRegion(uint64_t channel, uint64_t rank, uint64_t bank,
                           VirtualRegionLoc& hotVirt, PhysicalRegionLoc& hotPhys,
                           double& hotScore);
    bool FindColdFastRegionInBank(uint64_t channel, uint64_t rank, uint64_t bank,
                                  const PhysicalRegionLoc& sourcePhys,
                                  VirtualRegionLoc& victimVirt,
                                  PhysicalRegionLoc& victimPhys,
                                  double& coldScore);
    bool FindColdFastRegionInRank(uint64_t channel, uint64_t rank, uint64_t sourceBank,
                                  const PhysicalRegionLoc& sourcePhys,
                                  VirtualRegionLoc& victimVirt,
                                  PhysicalRegionLoc& victimPhys,
                                  double& coldScore);
    MigrationCandidate BuildCandidate(const VirtualRegionLoc& sourceVirt,
                                      const PhysicalRegionLoc& sourcePhys,
                                      double hotScore,
                                      const VirtualRegionLoc& victimVirt,
                                      const PhysicalRegionLoc& victimPhys,
                                      double coldScore,
                                      bool interbank) const;
    MigrationCandidate ChooseMigrationCandidate(const MigrationCandidate& intra,
                                                const MigrationCandidate& inter) const;

    inline uint64_t MakeKey(uint64_t channel, uint64_t rank,
                            uint64_t bank, uint64_t VRN) const {
        return ((((channel * numRanks) + rank) * numBanks) + bank) * numRegionsPerBank
               + VRN;
    }

    inline MigrationCandidate MakeInvalidCandidate() const {
        MigrationCandidate candidate;
        candidate.valid = false;
        candidate.interbank = false;
        candidate.hotScore = 0.0;
        candidate.coldScore = 0.0;
        candidate.rawBenefit = 0.0;
        candidate.migrationCost = 0.0;
        candidate.weightedBenefit = 0.0;
        return candidate;
    }

    ncounter_t totalMigrations;
    ncounter_t totalEpochs;
    ncounter_t migrationsPerBank[8];
    ncounter_t hotAccessesToFast;
    ncounter_t hotAccessesToSlow;
    ncounter_t totalInterBankMigrations;
    ncounter_t totalIntraBankMigrations;
    ncounter_t interBankMigrationAttempts;
    ncounter_t interBankMigrationRejects;
    ncounter_t cooldownBlockedMigrations;

    double avgScoreDifference;
    double maxScoreDifference;
};

}; // namespace NVM

#endif // __RERAM_REGION_CONTROLLER_H__
