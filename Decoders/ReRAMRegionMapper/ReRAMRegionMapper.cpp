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

#include "Decoders/ReRAMRegionMapper/ReRAMRegionMapper.h"
#include <iostream>
#include <fstream>
#include <cassert>

using namespace NVM;

ReRAMRegionMapper::ReRAMRegionMapper()
    : AddressTranslator()
{
    regionSwaps = 0;
    fastRegionAccesses = 0;
    slowRegionAccesses = 0;
    totalTranslations = 0;

    numChannels = 1;
    numRanks = 1;
    regionSize = 64;
    numRegionsPerBank = 1024;
    numRegionsPerMat = 16;
    matHeight = 1024;
    numMats = 64;
    fastRegionsPerMat = 4;
    numBanks = 8;
    numRows = 65536;
}

ReRAMRegionMapper::~ReRAMRegionMapper()
{
    regionTable.clear();
    inverseRegionTable.clear();
}

void ReRAMRegionMapper::SetConfig(Config *config, bool createChildren)
{
    AddressTranslator::SetConfig(config, createChildren);

    if (config->KeyExists("CHANNELS"))
        numChannels = config->GetValue("CHANNELS");

    if (config->KeyExists("RANKS"))
        numRanks = config->GetValue("RANKS");

    if (config->KeyExists("BANKS"))
        numBanks = config->GetValue("BANKS");

    if (config->KeyExists("ROWS"))
        numRows = config->GetValue("ROWS");

    if (config->KeyExists("MATHeight"))
        matHeight = config->GetValue("MATHeight");

    if (config->KeyExists("RegionSize"))
        regionSize = config->GetValue("RegionSize");
    else
        regionSize = 64;

    if (config->KeyExists("FastRegionsPerMat"))
        fastRegionsPerMat = config->GetValue("FastRegionsPerMat");
    else
        fastRegionsPerMat = 4;

    numMats = numRows / matHeight;
    numRegionsPerBank = numRows / regionSize;
    numRegionsPerMat = matHeight / regionSize;

    assert(regionSize > 0 && "RegionSize must be positive");
    assert((regionSize & (regionSize - 1)) == 0 && "RegionSize must be power of 2");
    assert(numRows % regionSize == 0 && "ROWS must be divisible by RegionSize");
    assert(matHeight % regionSize == 0 && "MATHeight must be divisible by RegionSize");
    assert(fastRegionsPerMat <= numRegionsPerMat &&
           "FastRegionsPerMat cannot exceed total regions per mat");

    InitializeRegionTable();

    std::cout << "ReRAMRegionMapper Configuration:" << std::endl;
    std::cout << "  Channels: " << numChannels << std::endl;
    std::cout << "  Ranks per channel: " << numRanks << std::endl;
    std::cout << "  Banks per rank: " << numBanks << std::endl;
    std::cout << "  Rows per bank: " << numRows << std::endl;
    std::cout << "  Mat height: " << matHeight << std::endl;
    std::cout << "  Mats per bank: " << numMats << std::endl;
    std::cout << "  Region size: " << regionSize << " rows" << std::endl;
    std::cout << "  Regions per bank: " << numRegionsPerBank << std::endl;
    std::cout << "  Regions per mat: " << numRegionsPerMat << std::endl;
    std::cout << "  Fast regions per mat: " << fastRegionsPerMat
              << " (" << (fastRegionsPerMat * 100 / numRegionsPerMat) << "%)" << std::endl;
    std::cout << "  Total fast regions per bank: " << (numMats * fastRegionsPerMat) << std::endl;
    std::cout << "  Region table size: "
              << (numChannels * numRanks * numBanks * numRegionsPerBank)
              << " entries" << std::endl;
}

void ReRAMRegionMapper::InitializeRegionTable()
{
    regionTable.clear();
    inverseRegionTable.clear();

    for (uint64_t channel = 0; channel < numChannels; channel++) {
        for (uint64_t rank = 0; rank < numRanks; rank++) {
            for (uint64_t bank = 0; bank < numBanks; bank++) {
                for (uint64_t VRN = 0; VRN < numRegionsPerBank; VRN++) {
                    regionTable[MakeVirtualKey(channel, rank, bank, VRN)] =
                        MakePhysicalRegionLoc(channel, rank, bank, VRN);
                    inverseRegionTable[MakePhysicalKey(channel, rank, bank, VRN)] =
                        MakeVirtualRegionLoc(channel, rank, bank, VRN);
                }
            }
        }
    }

    std::cout << "ReRAMRegionMapper: Initialized "
              << (numChannels * numRanks * numBanks * numRegionsPerBank)
              << " region mappings (identity)" << std::endl;
}

void ReRAMRegionMapper::Translate(uint64_t address,
                                   uint64_t *row, uint64_t *col,
                                   uint64_t *bank, uint64_t *rank,
                                   uint64_t *channel, uint64_t *subarray)
{
    AddressTranslator::Translate(address, row, col, bank, rank, channel, subarray);

    uint64_t virtChannel = *channel;
    uint64_t virtRank = *rank;
    uint64_t virtBank = *bank;
    uint64_t VRA = *row;
    uint64_t VRN = GetVRN(VRA);
    uint64_t RO = GetRegionOffset(VRA);

    PhysicalRegionLoc phys = GetPhysicalLocation(virtChannel, virtRank, virtBank, VRN);
    uint64_t PRA = GetPRA(phys.prn, RO);

    *channel = phys.channel;
    *rank = phys.rank;
    *bank = phys.bank;
    *row = PRA;
    *subarray = PRA >> MAT_SHIFT;

    totalTranslations++;
    if (IsFastRegion(phys)) {
        fastRegionAccesses++;
    } else {
        slowRegionAccesses++;
    }
}

bool ReRAMRegionMapper::SwapRegions(const VirtualRegionLoc& source,
                                    const VirtualRegionLoc& victim)
{
    assert(source.channel < numChannels && victim.channel < numChannels);
    assert(source.rank < numRanks && victim.rank < numRanks);
    assert(source.bank < numBanks && victim.bank < numBanks);
    assert(source.vrn < numRegionsPerBank && victim.vrn < numRegionsPerBank);

    if (source.channel != victim.channel || source.rank != victim.rank) {
        return false;
    }

    if (source.bank == victim.bank && source.vrn == victim.vrn) {
        return false;
    }

    uint64_t sourceKey = MakeVirtualKey(source.channel, source.rank, source.bank, source.vrn);
    uint64_t victimKey = MakeVirtualKey(victim.channel, victim.rank, victim.bank, victim.vrn);

    PhysicalRegionLoc sourcePhys = regionTable[sourceKey];
    PhysicalRegionLoc victimPhys = regionTable[victimKey];

    regionTable[sourceKey] = victimPhys;
    regionTable[victimKey] = sourcePhys;

    inverseRegionTable[MakePhysicalKey(sourcePhys.channel, sourcePhys.rank,
                                       sourcePhys.bank, sourcePhys.prn)] = victim;
    inverseRegionTable[MakePhysicalKey(victimPhys.channel, victimPhys.rank,
                                       victimPhys.bank, victimPhys.prn)] = source;

    regionSwaps++;

    #ifdef DEBUG_REGION_MAPPER
    std::cout << "ReRAMRegionMapper: Swapped regions within channel " << source.channel
              << ", rank " << source.rank << std::endl;
    #endif

    return true;
}

bool ReRAMRegionMapper::IsFastRegion(uint64_t channel, uint64_t rank,
                                     uint64_t bank, uint64_t PRN) const
{
    (void)channel;
    (void)rank;
    (void)bank;
    uint64_t regionInMat = PRN % numRegionsPerMat;
    return (regionInMat < fastRegionsPerMat);
}

bool ReRAMRegionMapper::IsFastRegion(const PhysicalRegionLoc& loc) const
{
    return IsFastRegion(loc.channel, loc.rank, loc.bank, loc.prn);
}

VirtualRegionLoc ReRAMRegionMapper::GetVirtualOwner(uint64_t channel, uint64_t rank,
                                                    uint64_t bank, uint64_t PRN) const
{
    assert(channel < numChannels && rank < numRanks && bank < numBanks);
    assert(PRN < numRegionsPerBank && "Invalid PRN");

    uint64_t key = MakePhysicalKey(channel, rank, bank, PRN);
    std::map<uint64_t, VirtualRegionLoc>::const_iterator it = inverseRegionTable.find(key);

    if (it != inverseRegionTable.end()) {
        return it->second;
    }

    std::cerr << "WARNING: ReRAMRegionMapper: Inverse lookup failed for "
              << "channel=" << channel << ", rank=" << rank
              << ", bank=" << bank << ", PRN=" << PRN
              << ". Using identity mapping." << std::endl;
    return MakeVirtualRegionLoc(channel, rank, bank, PRN);
}

PhysicalRegionLoc ReRAMRegionMapper::GetPhysicalLocation(uint64_t channel, uint64_t rank,
                                                         uint64_t bank, uint64_t VRN) const
{
    assert(channel < numChannels && rank < numRanks && bank < numBanks);
    assert(VRN < numRegionsPerBank && "Invalid VRN");

    uint64_t key = MakeVirtualKey(channel, rank, bank, VRN);
    std::map<uint64_t, PhysicalRegionLoc>::const_iterator it = regionTable.find(key);

    if (it != regionTable.end()) {
        return it->second;
    }

    std::cerr << "WARNING: ReRAMRegionMapper: Region lookup failed for "
              << "channel=" << channel << ", rank=" << rank
              << ", bank=" << bank << ", VRN=" << VRN
              << ". Using identity mapping." << std::endl;
    return MakePhysicalRegionLoc(channel, rank, bank, VRN);
}

void ReRAMRegionMapper::RegisterStats()
{
    AddressTranslator::RegisterStats();

    AddStat(regionSwaps);
    AddStat(fastRegionAccesses);
    AddStat(slowRegionAccesses);
    AddStat(totalTranslations);
}

void ReRAMRegionMapper::CalculateStats()
{
    if (totalTranslations > 0) {
        double fastRatio = (double)fastRegionAccesses / totalTranslations;
        double slowRatio = (double)slowRegionAccesses / totalTranslations;

        std::cout << "ReRAMRegionMapper Statistics:" << std::endl;
        std::cout << "  Total translations: " << totalTranslations << std::endl;
        std::cout << "  Fast region accesses: " << fastRegionAccesses
                  << " (" << (fastRatio * 100) << "%)" << std::endl;
        std::cout << "  Slow region accesses: " << slowRegionAccesses
                  << " (" << (slowRatio * 100) << "%)" << std::endl;
        std::cout << "  Region swaps: " << regionSwaps << std::endl;
        std::cout << "  Average swaps per 1M translations: "
                  << (regionSwaps * 1000000.0 / totalTranslations) << std::endl;
    }
}

void ReRAMRegionMapper::CreateCheckpoint(std::string dir)
{
    std::string filename = dir + "/ReRAMRegionMapper.checkpoint";
    std::ofstream checkpoint(filename.c_str(), std::ios::binary);

    if (!checkpoint.is_open()) {
        std::cerr << "ERROR: Could not create checkpoint file: " << filename << std::endl;
        return;
    }

    checkpoint.write(reinterpret_cast<const char*>(&numChannels), sizeof(numChannels));
    checkpoint.write(reinterpret_cast<const char*>(&numRanks), sizeof(numRanks));
    checkpoint.write(reinterpret_cast<const char*>(&numBanks), sizeof(numBanks));
    checkpoint.write(reinterpret_cast<const char*>(&numRegionsPerBank), sizeof(numRegionsPerBank));

    uint64_t tableSize = regionTable.size();
    checkpoint.write(reinterpret_cast<const char*>(&tableSize), sizeof(tableSize));

    for (std::map<uint64_t, PhysicalRegionLoc>::const_iterator it = regionTable.begin();
         it != regionTable.end(); ++it) {
        checkpoint.write(reinterpret_cast<const char*>(&it->first), sizeof(it->first));
        checkpoint.write(reinterpret_cast<const char*>(&it->second.channel), sizeof(it->second.channel));
        checkpoint.write(reinterpret_cast<const char*>(&it->second.rank), sizeof(it->second.rank));
        checkpoint.write(reinterpret_cast<const char*>(&it->second.bank), sizeof(it->second.bank));
        checkpoint.write(reinterpret_cast<const char*>(&it->second.prn), sizeof(it->second.prn));
    }

    checkpoint.write(reinterpret_cast<const char*>(&regionSwaps), sizeof(regionSwaps));
    checkpoint.write(reinterpret_cast<const char*>(&fastRegionAccesses), sizeof(fastRegionAccesses));
    checkpoint.write(reinterpret_cast<const char*>(&slowRegionAccesses), sizeof(slowRegionAccesses));
    checkpoint.write(reinterpret_cast<const char*>(&totalTranslations), sizeof(totalTranslations));

    checkpoint.close();
    std::cout << "ReRAMRegionMapper: Checkpoint saved to " << filename << std::endl;
}

void ReRAMRegionMapper::RestoreCheckpoint(std::string dir)
{
    std::string filename = dir + "/ReRAMRegionMapper.checkpoint";
    std::ifstream checkpoint(filename.c_str(), std::ios::binary);

    if (!checkpoint.is_open()) {
        std::cerr << "WARNING: Could not open checkpoint file: " << filename << std::endl;
        return;
    }

    uint64_t savedNumChannels, savedNumRanks, savedNumBanks, savedNumRegionsPerBank;
    checkpoint.read(reinterpret_cast<char*>(&savedNumChannels), sizeof(savedNumChannels));
    checkpoint.read(reinterpret_cast<char*>(&savedNumRanks), sizeof(savedNumRanks));
    checkpoint.read(reinterpret_cast<char*>(&savedNumBanks), sizeof(savedNumBanks));
    checkpoint.read(reinterpret_cast<char*>(&savedNumRegionsPerBank), sizeof(savedNumRegionsPerBank));

    if (savedNumChannels != numChannels || savedNumRanks != numRanks ||
        savedNumBanks != numBanks || savedNumRegionsPerBank != numRegionsPerBank) {
        std::cerr << "ERROR: Checkpoint configuration mismatch!" << std::endl;
        checkpoint.close();
        return;
    }

    regionTable.clear();
    inverseRegionTable.clear();

    uint64_t tableSize;
    checkpoint.read(reinterpret_cast<char*>(&tableSize), sizeof(tableSize));

    for (uint64_t i = 0; i < tableSize; i++) {
        uint64_t key;
        PhysicalRegionLoc value;
        checkpoint.read(reinterpret_cast<char*>(&key), sizeof(key));
        checkpoint.read(reinterpret_cast<char*>(&value.channel), sizeof(value.channel));
        checkpoint.read(reinterpret_cast<char*>(&value.rank), sizeof(value.rank));
        checkpoint.read(reinterpret_cast<char*>(&value.bank), sizeof(value.bank));
        checkpoint.read(reinterpret_cast<char*>(&value.prn), sizeof(value.prn));
        regionTable[key] = value;

        uint64_t regionIndex = key % numRegionsPerBank;
        uint64_t container = key / numRegionsPerBank;
        uint64_t bank = container % numBanks;
        container /= numBanks;
        uint64_t rank = container % numRanks;
        uint64_t channel = container / numRanks;

        inverseRegionTable[MakePhysicalKey(value.channel, value.rank, value.bank, value.prn)] =
            MakeVirtualRegionLoc(channel, rank, bank, regionIndex);
    }

    checkpoint.read(reinterpret_cast<char*>(&regionSwaps), sizeof(regionSwaps));
    checkpoint.read(reinterpret_cast<char*>(&fastRegionAccesses), sizeof(fastRegionAccesses));
    checkpoint.read(reinterpret_cast<char*>(&slowRegionAccesses), sizeof(slowRegionAccesses));
    checkpoint.read(reinterpret_cast<char*>(&totalTranslations), sizeof(totalTranslations));

    checkpoint.close();
    std::cout << "ReRAMRegionMapper: Checkpoint restored from " << filename << std::endl;
}
