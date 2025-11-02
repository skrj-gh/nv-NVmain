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
    // Initialize statistics counters
    regionSwaps = 0;
    fastRegionAccesses = 0;
    slowRegionAccesses = 0;
    totalTranslations = 0;

    // Default configuration values (will be overridden by SetConfig)
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
    // Clear region tables
    regionTable.clear();
    inverseRegionTable.clear();
}

void ReRAMRegionMapper::SetConfig(Config *config, bool createChildren)
{
    // Call parent SetConfig first
    AddressTranslator::SetConfig(config, createChildren);

    // Read configuration parameters
    if (config->KeyExists("BANKS"))
        numBanks = config->GetValue("BANKS");

    if (config->KeyExists("ROWS"))
        numRows = config->GetValue("ROWS");

    if (config->KeyExists("MATHeight"))
        matHeight = config->GetValue("MATHeight");

    if (config->KeyExists("RegionSize"))
        regionSize = config->GetValue("RegionSize");
    else
        regionSize = 64; // Default: 64 rows per region

    if (config->KeyExists("FastRegionsPerMat"))
        fastRegionsPerMat = config->GetValue("FastRegionsPerMat");
    else
        fastRegionsPerMat = 4; // Default: 25% fast regions

    // Calculate derived parameters
    numMats = numRows / matHeight;
    numRegionsPerBank = numRows / regionSize;
    numRegionsPerMat = matHeight / regionSize;

    // Validation checks
    assert(regionSize > 0 && "RegionSize must be positive");
    assert((regionSize & (regionSize - 1)) == 0 && "RegionSize must be power of 2");
    assert(numRows % regionSize == 0 && "ROWS must be divisible by RegionSize");
    assert(matHeight % regionSize == 0 && "MATHeight must be divisible by RegionSize");
    assert(fastRegionsPerMat <= numRegionsPerMat &&
           "FastRegionsPerMat cannot exceed total regions per mat");

    // Initialize region tables with identity mapping
    InitializeRegionTable();

    // Print configuration summary
    std::cout << "ReRAMRegionMapper Configuration:" << std::endl;
    std::cout << "  Banks: " << numBanks << std::endl;
    std::cout << "  Rows per bank: " << numRows << std::endl;
    std::cout << "  Mat height: " << matHeight << std::endl;
    std::cout << "  Mats per bank: " << numMats << std::endl;
    std::cout << "  Region size: " << regionSize << " rows" << std::endl;
    std::cout << "  Regions per bank: " << numRegionsPerBank << std::endl;
    std::cout << "  Regions per mat: " << numRegionsPerMat << std::endl;
    std::cout << "  Fast regions per mat: " << fastRegionsPerMat
              << " (" << (fastRegionsPerMat * 100 / numRegionsPerMat) << "%)" << std::endl;
    std::cout << "  Total fast regions per bank: " << (numMats * fastRegionsPerMat) << std::endl;
    std::cout << "  Region table size: " << (numBanks * numRegionsPerBank)
              << " entries (~" << (numBanks * numRegionsPerBank * 10 / 8192) << " KB)" << std::endl;
}

void ReRAMRegionMapper::InitializeRegionTable()
{
    // Initialize with identity mapping: VRN → VRN (no translation initially)
    for (uint64_t bank = 0; bank < numBanks; bank++) {
        for (uint64_t VRN = 0; VRN < numRegionsPerBank; VRN++) {
            uint64_t key = MakeKey(bank, VRN);

            // Forward table: VRN → PRN (identity: PRN = VRN initially)
            regionTable[key] = VRN;

            // Inverse table: PRN → VRN (identity: VRN = PRN initially)
            inverseRegionTable[key] = VRN;
        }
    }

    std::cout << "ReRAMRegionMapper: Initialized " << (numBanks * numRegionsPerBank)
              << " region mappings (identity)" << std::endl;
}

void ReRAMRegionMapper::Translate(uint64_t address,
                                   uint64_t *row, uint64_t *col,
                                   uint64_t *bank, uint64_t *rank,
                                   uint64_t *channel, uint64_t *subarray)
{
    // Step 1: Call parent to perform standard address decomposition
    // This extracts channel, rank, bank, column, and VRA (in *row)
    AddressTranslator::Translate(address, row, col, bank, rank, channel, subarray);

    // Step 2: Extract Virtual Row Address (VRA) from row field
    uint64_t VRA = *row;

    // Step 3: Split VRA into Virtual Region Number (VRN) and Region Offset (RO)
    uint64_t VRN = GetVRN(VRA);      // Top 10 bits: VRA >> 6
    uint64_t RO = GetRegionOffset(VRA);  // Bottom 6 bits: VRA & 0x3F

    // Step 4: Lookup Region Table to get Physical Region Number (PRN)
    uint64_t key = MakeKey(*bank, VRN);
    uint64_t PRN;

    auto it = regionTable.find(key);
    if (it != regionTable.end()) {
        PRN = it->second;
    } else {
        // If not found (shouldn't happen with proper initialization), use identity
        PRN = VRN;
        std::cerr << "WARNING: ReRAMRegionMapper: Region table lookup failed for "
                  << "bank=" << *bank << ", VRN=" << VRN << ". Using identity mapping."
                  << std::endl;
    }

    // Step 5: Reconstruct Physical Row Address (PRA)
    uint64_t PRA = GetPRA(PRN, RO);  // (PRN << 6) | RO

    // Step 6: Update row with translated address
    *row = PRA;

    // Step 7: Update subarray (mat index) based on PRA
    // Mat index = top 6 bits of PRA
    *subarray = PRA >> MAT_SHIFT;

    // Step 8: Update statistics
    totalTranslations++;
    if (IsFastRegion(PRN)) {
        fastRegionAccesses++;
    } else {
        slowRegionAccesses++;
    }
}

void ReRAMRegionMapper::SwapRegions(uint64_t bank, uint64_t VRN_hot, uint64_t VRN_cold)
{
    // Validate inputs
    assert(bank < numBanks && "Invalid bank ID");
    assert(VRN_hot < numRegionsPerBank && "Invalid VRN_hot");
    assert(VRN_cold < numRegionsPerBank && "Invalid VRN_cold");
    assert(VRN_hot != VRN_cold && "Cannot swap region with itself");

    // Generate keys for region table lookup
    uint64_t key_hot = MakeKey(bank, VRN_hot);
    uint64_t key_cold = MakeKey(bank, VRN_cold);

    // Get current Physical Region Numbers for both virtual regions
    uint64_t PRN_hot = regionTable[key_hot];
    uint64_t PRN_cold = regionTable[key_cold];

    // Swap forward mappings: VRN → PRN
    regionTable[key_hot] = PRN_cold;   // Hot VRN now maps to cold's physical region
    regionTable[key_cold] = PRN_hot;   // Cold VRN now maps to hot's physical region

    // Update inverse mappings: PRN → VRN
    uint64_t inv_key_hot = MakeKey(bank, PRN_hot);
    uint64_t inv_key_cold = MakeKey(bank, PRN_cold);
    inverseRegionTable[inv_key_hot] = VRN_cold;
    inverseRegionTable[inv_key_cold] = VRN_hot;

    // Update statistics
    regionSwaps++;

    // Debug output (can be disabled in production)
    #ifdef DEBUG_REGION_MAPPER
    std::cout << "ReRAMRegionMapper: Swapped regions in bank " << bank << ":" << std::endl;
    std::cout << "  VRN " << VRN_hot << " (hot): PRN " << PRN_hot << " → " << PRN_cold
              << " (fast=" << IsFastRegion(PRN_cold) << ")" << std::endl;
    std::cout << "  VRN " << VRN_cold << " (cold): PRN " << PRN_cold << " → " << PRN_hot
              << " (fast=" << IsFastRegion(PRN_hot) << ")" << std::endl;
    #endif
}

bool ReRAMRegionMapper::IsFastRegion(uint64_t PRN) const
{
    // Decompose PRN into mat index and region within mat
    // PRN range: 0-1023 (1024 total regions per bank)
    // With 64 mats per bank: 16 regions per mat

    uint64_t regionInMat = PRN % numRegionsPerMat;  // PRN % 16

    // First fastRegionsPerMat regions in each mat are fast (near wordline driver)
    // Example with fastRegionsPerMat = 4:
    //   Mat 0: PRN 0-3 → fast,    PRN 4-15 → slow
    //   Mat 1: PRN 16-19 → fast,  PRN 20-31 → slow
    //   Mat 2: PRN 32-35 → fast,  PRN 36-47 → slow
    //   etc.

    return (regionInMat < fastRegionsPerMat);
}

uint64_t ReRAMRegionMapper::GetVRNFromPRN(uint64_t bank, uint64_t PRN) const
{
    assert(bank < numBanks && "Invalid bank ID");
    assert(PRN < numRegionsPerBank && "Invalid PRN");

    uint64_t key = MakeKey(bank, PRN);

    auto it = inverseRegionTable.find(key);
    if (it != inverseRegionTable.end()) {
        return it->second;
    } else {
        // If not found, use identity (shouldn't happen with proper initialization)
        std::cerr << "WARNING: ReRAMRegionMapper: Inverse lookup failed for "
                  << "bank=" << bank << ", PRN=" << PRN << ". Using identity mapping."
                  << std::endl;
        return PRN;
    }
}

uint64_t ReRAMRegionMapper::GetPRN(uint64_t bank, uint64_t VRN) const
{
    assert(bank < numBanks && "Invalid bank ID");
    assert(VRN < numRegionsPerBank && "Invalid VRN");

    uint64_t key = MakeKey(bank, VRN);

    auto it = regionTable.find(key);
    if (it != regionTable.end()) {
        return it->second;
    } else {
        // If not found, use identity
        std::cerr << "WARNING: ReRAMRegionMapper: Region lookup failed for "
                  << "bank=" << bank << ", VRN=" << VRN << ". Using identity mapping."
                  << std::endl;
        return VRN;
    }
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
    // Calculate fast/slow region access ratio
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

    // Write configuration parameters
    checkpoint.write(reinterpret_cast<const char*>(&numBanks), sizeof(numBanks));
    checkpoint.write(reinterpret_cast<const char*>(&numRegionsPerBank), sizeof(numRegionsPerBank));

    // Write region table size
    uint64_t tableSize = regionTable.size();
    checkpoint.write(reinterpret_cast<const char*>(&tableSize), sizeof(tableSize));

    // Write region table entries
    for (const auto& entry : regionTable) {
        checkpoint.write(reinterpret_cast<const char*>(&entry.first), sizeof(entry.first));
        checkpoint.write(reinterpret_cast<const char*>(&entry.second), sizeof(entry.second));
    }

    // Write statistics
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

    // Read configuration parameters
    uint64_t savedNumBanks, savedNumRegionsPerBank;
    checkpoint.read(reinterpret_cast<char*>(&savedNumBanks), sizeof(savedNumBanks));
    checkpoint.read(reinterpret_cast<char*>(&savedNumRegionsPerBank), sizeof(savedNumRegionsPerBank));

    // Validate configuration matches
    if (savedNumBanks != numBanks || savedNumRegionsPerBank != numRegionsPerBank) {
        std::cerr << "ERROR: Checkpoint configuration mismatch!" << std::endl;
        std::cerr << "  Expected: banks=" << numBanks << ", regions=" << numRegionsPerBank << std::endl;
        std::cerr << "  Found: banks=" << savedNumBanks << ", regions=" << savedNumRegionsPerBank << std::endl;
        checkpoint.close();
        return;
    }

    // Clear existing tables
    regionTable.clear();
    inverseRegionTable.clear();

    // Read region table size
    uint64_t tableSize;
    checkpoint.read(reinterpret_cast<char*>(&tableSize), sizeof(tableSize));

    // Read region table entries
    for (uint64_t i = 0; i < tableSize; i++) {
        uint64_t key, value;
        checkpoint.read(reinterpret_cast<char*>(&key), sizeof(key));
        checkpoint.read(reinterpret_cast<char*>(&value), sizeof(value));
        regionTable[key] = value;

        // Rebuild inverse table
        uint64_t bank = key >> 10;
        uint64_t VRN = key & 0x3FF;
        uint64_t PRN = value;
        uint64_t inv_key = MakeKey(bank, PRN);
        inverseRegionTable[inv_key] = VRN;
    }

    // Read statistics
    checkpoint.read(reinterpret_cast<char*>(&regionSwaps), sizeof(regionSwaps));
    checkpoint.read(reinterpret_cast<char*>(&fastRegionAccesses), sizeof(fastRegionAccesses));
    checkpoint.read(reinterpret_cast<char*>(&slowRegionAccesses), sizeof(slowRegionAccesses));
    checkpoint.read(reinterpret_cast<char*>(&totalTranslations), sizeof(totalTranslations));

    checkpoint.close();
    std::cout << "ReRAMRegionMapper: Checkpoint restored from " << filename << std::endl;
}
