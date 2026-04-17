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

struct VirtualRegionLoc
{
    uint64_t channel;
    uint64_t rank;
    uint64_t bank;
    uint64_t vrn;
};

struct PhysicalRegionLoc
{
    uint64_t channel;
    uint64_t rank;
    uint64_t bank;
    uint64_t prn;
};

/**
 * @class ReRAMRegionMapper
 * @brief Address translator implementing dynamic region-based mapping for ReRAM
 *
 * This decoder implements hardware-managed virtual-to-physical region mapping for
 * ReRAM wear leveling and latency optimization. It translates virtual regions
 * identified by channel, rank, bank, and row into their current physical
 * location. The physical destination may remain intrabank or migrate across
 * banks within the same rank.
 */
class ReRAMRegionMapper : public AddressTranslator
{
  public:
    ReRAMRegionMapper();
    ~ReRAMRegionMapper();

    void SetConfig(Config *config, bool createChildren = true);

    virtual void Translate(uint64_t address, uint64_t *row, uint64_t *col,
                          uint64_t *bank, uint64_t *rank, uint64_t *channel,
                          uint64_t *subarray);

    using AddressTranslator::Translate;

    bool SwapRegions(const VirtualRegionLoc& source, const VirtualRegionLoc& victim);

    bool IsFastRegion(uint64_t channel, uint64_t rank, uint64_t bank, uint64_t PRN) const;
    bool IsFastRegion(const PhysicalRegionLoc& loc) const;

    VirtualRegionLoc GetVirtualOwner(uint64_t channel, uint64_t rank,
                                     uint64_t bank, uint64_t PRN) const;
    PhysicalRegionLoc GetPhysicalLocation(uint64_t channel, uint64_t rank,
                                          uint64_t bank, uint64_t VRN) const;

    void RegisterStats();
    void CalculateStats();

    void CreateCheckpoint(std::string dir);
    void RestoreCheckpoint(std::string dir);

  private:
    std::map<uint64_t, PhysicalRegionLoc> regionTable;
    std::map<uint64_t, VirtualRegionLoc> inverseRegionTable;

    uint64_t numChannels;
    uint64_t numRanks;
    uint64_t regionSize;
    uint64_t numRegionsPerBank;
    uint64_t numRegionsPerMat;
    uint64_t matHeight;
    uint64_t numMats;
    uint64_t fastRegionsPerMat;
    uint64_t numBanks;
    uint64_t numRows;

    static const int VRN_SHIFT = 6;
    static const int MAT_SHIFT = 10;
    static const uint64_t RO_MASK = 0x3F;

    void InitializeRegionTable();

    inline uint64_t GetVRN(uint64_t VRA) const {
        return VRA >> VRN_SHIFT;
    }

    inline uint64_t GetRegionOffset(uint64_t VRA) const {
        return VRA & RO_MASK;
    }

    inline uint64_t GetPRA(uint64_t PRN, uint64_t RO) const {
        return (PRN << VRN_SHIFT) | RO;
    }

    inline uint64_t MakeVirtualKey(uint64_t channel, uint64_t rank,
                                   uint64_t bank, uint64_t regionNum) const {
        return ((((channel * numRanks) + rank) * numBanks) + bank) * numRegionsPerBank
               + regionNum;
    }

    inline uint64_t MakePhysicalKey(uint64_t channel, uint64_t rank,
                                    uint64_t bank, uint64_t regionNum) const {
        return ((((channel * numRanks) + rank) * numBanks) + bank) * numRegionsPerBank
               + regionNum;
    }

    inline VirtualRegionLoc MakeVirtualRegionLoc(uint64_t channel, uint64_t rank,
                                                 uint64_t bank, uint64_t vrn) const {
        VirtualRegionLoc loc;
        loc.channel = channel;
        loc.rank = rank;
        loc.bank = bank;
        loc.vrn = vrn;
        return loc;
    }

    inline PhysicalRegionLoc MakePhysicalRegionLoc(uint64_t channel, uint64_t rank,
                                                   uint64_t bank, uint64_t prn) const {
        PhysicalRegionLoc loc;
        loc.channel = channel;
        loc.rank = rank;
        loc.bank = bank;
        loc.prn = prn;
        return loc;
    }

    ncounter_t regionSwaps;
    ncounter_t fastRegionAccesses;
    ncounter_t slowRegionAccesses;
    ncounter_t totalTranslations;
};

}; // namespace NVM

#endif // __RERAM_REGION_MAPPER_H__
