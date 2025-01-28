#pragma once
//#define INT32_BITSMASK 0xFFFFFFFF

struct IdxPair {  
     int i , j ; 
} ; 

struct VertPair_Hash {

    size_t operator()(const size_t vert_pair) const noexcept {
        // since the pair is ordered it is litterally never the same, so might as
        // well return it as is
        return vert_pair;
    }

    //
};

static size_t ordered_pair64(const unsigned a, const unsigned b) noexcept {
    return (size_t(a < b ? a : b) << 32) | (a > b ? a : b);
}



