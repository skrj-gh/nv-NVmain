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

    alpha = 0.5;
    beta = 0.5;
    epochLength = 1000000;
    migrationThreshold = 100.0;
    enableInterBankMigration = true;
    interBankMigrationThreshold = 180.0;
    interBankMigrationCost = 80.0;
    interBankScoreWeight = 0.8;
    intraBankScoreWeight = 1.0;
    maxInterBankMigrationsPerEpoch = 1;
    maxTotalMigrationsPerEpoch = 2;
    interBankCooldownEpochs = 2;
    preferIntraBankOnTie = true;
    allowCrossRankMigration = false;
    allowCrossChannelMigration = false;

    numChannels = 2;
    numRanks = 2;
    numBanks = 8;
    numRegionsPerBank = 1024;
    numRegionsPerMat = 16;
    fastRegionsPerMat = 4;

    currentCycle = 0;
    lastEpochCycle = 0;

    totalMigrations = 0;
    totalEpochs = 0;
    for (int i = 0; i < 8; i++) {
        migrationsPerBank[i] = 0;
    }
    hotAccessesToFast = 0;
    hotAccessesToSlow = 0;
    totalInterBankMigrations = 0;
    totalIntraBankMigrations = 0;
    interBankMigrationAttempts = 0;
    interBankMigrationRejects = 0;
    cooldownBlockedMigrations = 0;
    avgScoreDifference = 0.0;
    maxScoreDifference = 0.0;
}

ReRAMRegionController::~ReRAMRegionController()
{
    writeScores.clear();
    readScores.clear();
    regionScores.clear();
    lastMigrationEpoch.clear();
}

void ReRAMRegionController::SetConfig(Config *conf, bool createChildren)
{
    FRFCFS::SetConfig(conf, createChildren);

    if (conf->KeyExists("CHANNELS"))
        numChannels = conf->GetValue("CHANNELS");

    if (conf->KeyExists("RANKS"))
        numRanks = conf->GetValue("RANKS");

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
        alpha = conf->GetEnergy("Alpha");

    if (conf->KeyExists("Beta"))
        beta = conf->GetEnergy("Beta");

    if (conf->KeyExists("EpochLength"))
        epochLength = conf->GetValue("EpochLength");

    if (conf->KeyExists("MigrationThreshold"))
        migrationThreshold = conf->GetValue("MigrationThreshold");

    if (conf->KeyExists("EnableInterBankMigration"))
        enableInterBankMigration = conf->GetBool("EnableInterBankMigration");

    if (conf->KeyExists("InterBankMigrationThreshold"))
        interBankMigrationThreshold = conf->GetValue("InterBankMigrationThreshold");

    if (conf->KeyExists("InterBankMigrationCost"))
        interBankMigrationCost = conf->GetValue("InterBankMigrationCost");

    if (conf->KeyExists("InterBankScoreWeight"))
        interBankScoreWeight = conf->GetEnergy("InterBankScoreWeight");

    if (conf->KeyExists("IntraBankScoreWeight"))
        intraBankScoreWeight = conf->GetEnergy("IntraBankScoreWeight");

    if (conf->KeyExists("MaxInterBankMigrationsPerEpoch"))
        maxInterBankMigrationsPerEpoch = conf->GetValue("MaxInterBankMigrationsPerEpoch");

    if (conf->KeyExists("MaxTotalMigrationsPerEpoch"))
        maxTotalMigrationsPerEpoch = conf->GetValue("MaxTotalMigrationsPerEpoch");

    if (conf->KeyExists("InterBankCooldownEpochs"))
        interBankCooldownEpochs = conf->GetValue("InterBankCooldownEpochs");

    if (conf->KeyExists("PreferIntraBankOnTie"))
        preferIntraBankOnTie = conf->GetBool("PreferIntraBankOnTie");

    if (conf->KeyExists("AllowCrossRankMigration"))
        allowCrossRankMigration = conf->GetBool("AllowCrossRankMigration");

    if (conf->KeyExists("AllowCrossChannelMigration"))
        allowCrossChannelMigration = conf->GetBool("AllowCrossChannelMigration");

    assert(alpha + beta > 0.0 && "Alpha + Beta must be positive");
    assert(epochLength > 0 && "EpochLength must be positive");
    assert(migrationThreshold >= 0.0 && "MigrationThreshold must be non-negative");
    assert(!allowCrossRankMigration &&
           "Cross-rank migration is not supported in this implementation");
    assert(!allowCrossChannelMigration &&
           "Cross-channel migration is not supported in this implementation");

    std::cout << "ReRAMRegionController Configuration:" << std::endl;
    std::cout << "  Alpha (write weight): " << alpha << std::endl;
    std::cout << "  Beta (read weight): " << beta << std::endl;
    std::cout << "  Epoch length: " << epochLength << " cycles" << std::endl;
    std::cout << "  Migration threshold: " << migrationThreshold << std::endl;
    std::cout << "  Interbank migration enabled: " << enableInterBankMigration << std::endl;
    std::cout << "  Interbank migration threshold: " << interBankMigrationThreshold << std::endl;
    std::cout << "  Interbank migration cost: " << interBankMigrationCost << std::endl;
    std::cout << "  Interbank cooldown epochs: " << interBankCooldownEpochs << std::endl;
    std::cout << "  Allow cross-rank migration: " << allowCrossRankMigration << std::endl;
    std::cout << "  Allow cross-channel migration: " << allowCrossChannelMigration << std::endl;
    std::cout << "  Channels: " << numChannels << std::endl;
    std::cout << "  Ranks per channel: " << numRanks << std::endl;
    std::cout << "  Banks per rank: " << numBanks << std::endl;
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
    bool issued = FRFCFS::IssueCommand(req);

    if (issued) {
        UpdateRegionScores(req);
    }

    return issued;
}

void ReRAMRegionController::UpdateRegionScores(NVMainRequest *req)
{
    if (regionMapper == NULL) {
        return;
    }

    NVMAddress &addr = req->address;
    uint64_t physChannel = addr.GetChannel();
    uint64_t physRank = addr.GetRank();
    uint64_t physBank = addr.GetBank();
    uint64_t PRA = addr.GetRow();
    uint64_t PRN = PRA >> 6;

    VirtualRegionLoc owner = regionMapper->GetVirtualOwner(physChannel, physRank, physBank, PRN);
    uint64_t key = MakeKey(owner.channel, owner.rank, owner.bank, owner.vrn);

    OpType op = req->type;
    if (op == WRITE || op == WRITE_PRECHARGE) {
        writeScores[key]++;
    } else if (op == READ || op == READ_PRECHARGE) {
        readScores[key]++;
    }

    double score = alpha * writeScores[key] + beta * readScores[key];
    bool isFast = regionMapper->IsFastRegion(physChannel, physRank, physBank, PRN);
    if (score > migrationThreshold) {
        if (isFast) {
            hotAccessesToFast++;
        } else {
            hotAccessesToSlow++;
        }
    }
}

void ReRAMRegionController::Cycle(ncycle_t steps)
{
    currentCycle += steps;
    FRFCFS::Cycle(steps);

    if (currentCycle - lastEpochCycle >= epochLength) {
        totalEpochs++;

        uint64_t totalUsed = 0;
        uint64_t interbankUsed = 0;
        uint64_t channel = GetID();

        for (uint64_t rank = 0; rank < numRanks; rank++) {
            for (uint64_t bank = 0; bank < numBanks; bank++) {
                if (totalUsed >= maxTotalMigrationsPerEpoch) {
                    break;
                }
                Migration(channel, rank, bank, totalUsed, interbankUsed);
            }
        }

        lastEpochCycle = currentCycle;
    }
}

void ReRAMRegionController::CalculateRegionScores(uint64_t channel, uint64_t rank, uint64_t bank)
{
    for (uint64_t VRN = 0; VRN < numRegionsPerBank; VRN++) {
        uint64_t key = MakeKey(channel, rank, bank, VRN);
        uint64_t WS = writeScores[key];
        uint64_t RS = readScores[key];
        regionScores[key] = alpha * WS + beta * RS;
    }
}

bool ReRAMRegionController::IsCoolingDown(const VirtualRegionLoc& loc) const
{
    if (interBankCooldownEpochs == 0) {
        return false;
    }

    uint64_t key = MakeKey(loc.channel, loc.rank, loc.bank, loc.vrn);
    std::map<uint64_t, uint64_t>::const_iterator it = lastMigrationEpoch.find(key);

    if (it == lastMigrationEpoch.end()) {
        return false;
    }

    return (totalEpochs - it->second) < interBankCooldownEpochs;
}

bool ReRAMRegionController::FindHotSlowRegion(uint64_t channel, uint64_t rank, uint64_t bank,
                                              VirtualRegionLoc& hotVirt,
                                              PhysicalRegionLoc& hotPhys,
                                              double& hotScore)
{
    hotScore = -1.0;
    bool found = false;

    for (uint64_t VRN = 0; VRN < numRegionsPerBank; VRN++) {
        VirtualRegionLoc virt;
        virt.channel = channel;
        virt.rank = rank;
        virt.bank = bank;
        virt.vrn = VRN;

        if (IsCoolingDown(virt)) {
            cooldownBlockedMigrations++;
            continue;
        }

        uint64_t key = MakeKey(channel, rank, bank, VRN);
        double score = regionScores[key];
        PhysicalRegionLoc phys = regionMapper->GetPhysicalLocation(channel, rank, bank, VRN);

        if (regionMapper->IsFastRegion(phys)) {
            continue;
        }

        if (score > hotScore) {
            hotVirt = virt;
            hotPhys = phys;
            hotScore = score;
            found = true;
        }
    }

    return found;
}

bool ReRAMRegionController::FindColdFastRegionInBank(uint64_t channel, uint64_t rank, uint64_t bank,
                                                      const PhysicalRegionLoc& sourcePhys,
                                                      VirtualRegionLoc& victimVirt,
                                                      PhysicalRegionLoc& victimPhys,
                                                      double& coldScore)
{
    (void)sourcePhys;
    coldScore = std::numeric_limits<double>::max();
    bool found = false;

    for (uint64_t VRN = 0; VRN < numRegionsPerBank; VRN++) {
        VirtualRegionLoc virt;
        virt.channel = channel;
        virt.rank = rank;
        virt.bank = bank;
        virt.vrn = VRN;

        if (IsCoolingDown(virt)) {
            continue;
        }

        PhysicalRegionLoc phys = regionMapper->GetPhysicalLocation(channel, rank, bank, VRN);
        if (!regionMapper->IsFastRegion(phys)) {
            continue;
        }

        uint64_t key = MakeKey(channel, rank, bank, VRN);
        double score = regionScores[key];
        if (score < coldScore) {
            victimVirt = virt;
            victimPhys = phys;
            coldScore = score;
            found = true;
        }
    }

    return found;
}

bool ReRAMRegionController::FindColdFastRegionInRank(uint64_t channel, uint64_t rank,
                                                      uint64_t sourceBank,
                                                      const PhysicalRegionLoc& sourcePhys,
                                                      VirtualRegionLoc& victimVirt,
                                                      PhysicalRegionLoc& victimPhys,
                                                      double& coldScore)
{
    coldScore = std::numeric_limits<double>::max();
    bool found = false;

    for (uint64_t bank = 0; bank < numBanks; bank++) {
        if (bank == sourceBank) {
            continue;
        }

        for (uint64_t VRN = 0; VRN < numRegionsPerBank; VRN++) {
            VirtualRegionLoc virt;
            virt.channel = channel;
            virt.rank = rank;
            virt.bank = bank;
            virt.vrn = VRN;

            if (IsCoolingDown(virt)) {
                continue;
            }

            PhysicalRegionLoc phys = regionMapper->GetPhysicalLocation(channel, rank, bank, VRN);
            if (!regionMapper->IsFastRegion(phys)) {
                continue;
            }

            if (phys.bank == sourcePhys.bank) {
                continue;
            }

            uint64_t key = MakeKey(channel, rank, bank, VRN);
            double score = regionScores[key];
            if (score < coldScore) {
                victimVirt = virt;
                victimPhys = phys;
                coldScore = score;
                found = true;
            }
        }
    }

    return found;
}

MigrationCandidate ReRAMRegionController::BuildCandidate(const VirtualRegionLoc& sourceVirt,
                                                         const PhysicalRegionLoc& sourcePhys,
                                                         double hotScore,
                                                         const VirtualRegionLoc& victimVirt,
                                                         const PhysicalRegionLoc& victimPhys,
                                                         double coldScore,
                                                         bool interbank) const
{
    MigrationCandidate candidate = MakeInvalidCandidate();
    double threshold = interbank ? interBankMigrationThreshold : migrationThreshold;
    double weight = interbank ? interBankScoreWeight : intraBankScoreWeight;
    double cost = interbank ? interBankMigrationCost : 0.0;
    double rawBenefit = hotScore - coldScore;
    double weightedBenefit = weight * rawBenefit - cost;

    if (weightedBenefit < threshold) {
        return candidate;
    }

    candidate.valid = true;
    candidate.interbank = interbank;
    candidate.sourceVirt = sourceVirt;
    candidate.victimVirt = victimVirt;
    candidate.sourcePhys = sourcePhys;
    candidate.victimPhys = victimPhys;
    candidate.hotScore = hotScore;
    candidate.coldScore = coldScore;
    candidate.rawBenefit = rawBenefit;
    candidate.migrationCost = cost;
    candidate.weightedBenefit = weightedBenefit;
    return candidate;
}

MigrationCandidate ReRAMRegionController::ChooseMigrationCandidate(const MigrationCandidate& intra,
                                                                   const MigrationCandidate& inter) const
{
    if (!intra.valid) {
        return inter;
    }
    if (!inter.valid) {
        return intra;
    }

    if (preferIntraBankOnTie && intra.weightedBenefit == inter.weightedBenefit) {
        return intra;
    }

    return (inter.weightedBenefit > intra.weightedBenefit) ? inter : intra;
}

void ReRAMRegionController::Migration(uint64_t channel, uint64_t rank, uint64_t bank,
                                      uint64_t& totalUsed, uint64_t& interbankUsed)
{
    if (regionMapper == NULL) {
        std::cerr << "WARNING: Migration called but region mapper not set" << std::endl;
        return;
    }

    CalculateRegionScores(channel, rank, bank);

    VirtualRegionLoc hotVirt;
    PhysicalRegionLoc hotPhys;
    double hotScore = 0.0;
    if (!FindHotSlowRegion(channel, rank, bank, hotVirt, hotPhys, hotScore)) {
        return;
    }

    MigrationCandidate intra = MakeInvalidCandidate();
    VirtualRegionLoc intraVictimVirt;
    PhysicalRegionLoc intraVictimPhys;
    double intraColdScore = 0.0;
    if (FindColdFastRegionInBank(channel, rank, bank, hotPhys,
                                 intraVictimVirt, intraVictimPhys, intraColdScore)) {
        intra = BuildCandidate(hotVirt, hotPhys, hotScore,
                               intraVictimVirt, intraVictimPhys, intraColdScore,
                               false);
    }

    MigrationCandidate inter = MakeInvalidCandidate();
    if (enableInterBankMigration && interbankUsed < maxInterBankMigrationsPerEpoch) {
        interBankMigrationAttempts++;
        VirtualRegionLoc interVictimVirt;
        PhysicalRegionLoc interVictimPhys;
        double interColdScore = 0.0;
        if (FindColdFastRegionInRank(channel, rank, bank, hotPhys,
                                     interVictimVirt, interVictimPhys, interColdScore)) {
            inter = BuildCandidate(hotVirt, hotPhys, hotScore,
                                   interVictimVirt, interVictimPhys, interColdScore,
                                   true);
        }
        if (!inter.valid) {
            interBankMigrationRejects++;
        }
    }

    MigrationCandidate best = ChooseMigrationCandidate(intra, inter);
    if (!best.valid) {
        return;
    }

    if (best.interbank && interbankUsed >= maxInterBankMigrationsPerEpoch) {
        return;
    }

    if (!regionMapper->SwapRegions(best.sourceVirt, best.victimVirt)) {
        return;
    }

    totalMigrations++;
    totalUsed++;
    if (best.sourceVirt.bank < 8) {
        migrationsPerBank[best.sourceVirt.bank]++;
    }
    if (best.interbank) {
        totalInterBankMigrations++;
        interbankUsed++;
    } else {
        totalIntraBankMigrations++;
    }

    uint64_t sourceKey = MakeKey(best.sourceVirt.channel, best.sourceVirt.rank,
                                 best.sourceVirt.bank, best.sourceVirt.vrn);
    uint64_t victimKey = MakeKey(best.victimVirt.channel, best.victimVirt.rank,
                                 best.victimVirt.bank, best.victimVirt.vrn);
    lastMigrationEpoch[sourceKey] = totalEpochs;
    lastMigrationEpoch[victimKey] = totalEpochs;

    if (best.rawBenefit > maxScoreDifference) {
        maxScoreDifference = best.rawBenefit;
    }
    avgScoreDifference = (avgScoreDifference * (totalMigrations - 1) + best.rawBenefit)
                         / totalMigrations;

    HalveScores();
}

void ReRAMRegionController::HalveScores()
{
    for (std::map<uint64_t, uint64_t>::iterator it = writeScores.begin();
         it != writeScores.end(); ++it) {
        it->second = it->second / 2;
    }

    for (std::map<uint64_t, uint64_t>::iterator it = readScores.begin();
         it != readScores.end(); ++it) {
        it->second = it->second / 2;
    }

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
    AddStat(totalInterBankMigrations);
    AddStat(totalIntraBankMigrations);
    AddStat(interBankMigrationAttempts);
    AddStat(interBankMigrationRejects);
    AddStat(cooldownBlockedMigrations);
}

void ReRAMRegionController::CalculateStats()
{
    FRFCFS::CalculateStats();

    std::cout << "\nReRAMRegionController Statistics:" << std::endl;
    std::cout << "  Total epochs: " << totalEpochs << std::endl;
    std::cout << "  Total migrations: " << totalMigrations << std::endl;
    std::cout << "  Intrabank migrations: " << totalIntraBankMigrations << std::endl;
    std::cout << "  Interbank migrations: " << totalInterBankMigrations << std::endl;
    std::cout << "  Interbank migration attempts: " << interBankMigrationAttempts << std::endl;
    std::cout << "  Interbank migration rejects: " << interBankMigrationRejects << std::endl;
    std::cout << "  Cooldown blocks: " << cooldownBlockedMigrations << std::endl;

    if (totalEpochs > 0) {
        std::cout << "  Migrations per epoch: "
                  << (double)totalMigrations / totalEpochs << std::endl;
    }

    std::cout << "  Per-bank migrations:" << std::endl;
    for (uint64_t i = 0; i < numBanks && i < 8; i++) {
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
