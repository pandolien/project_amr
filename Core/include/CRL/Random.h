// 재현 가능한 난수 (PCG32). 같은 seed 면 항상 같은 수열이 나온다 -> 학습 결과도 재현된다.
#ifndef CORE_CRL_RANDOM_H
#define CORE_CRL_RANDOM_H

#include <stdint.h>

namespace simul {
namespace rl {

class Rng {
public:
    explicit Rng(uint64_t seed = 1) { Seed(seed); }

    void Seed(uint64_t seed) {
        state_ = 0;
        NextU32();
        state_ += seed;
        NextU32();
    }

    uint32_t NextU32() {
        const uint64_t old = state_;
        state_ = old * 6364136223846793005ULL + 1442695040888963407ULL;
        const uint32_t xorshifted = static_cast<uint32_t>(((old >> 18u) ^ old) >> 27u);
        const uint32_t rot = static_cast<uint32_t>(old >> 59u);
        return (xorshifted >> rot) | (xorshifted << ((~rot + 1u) & 31u));
    }

    // [0, 1)
    double Uniform() { return static_cast<double>(NextU32()) / 4294967296.0; }

    // [0, n)  (n > 0)
    int UniformInt(int n) {
        const int v = static_cast<int>(Uniform() * static_cast<double>(n));
        return v < n ? v : n - 1;
    }

private:
    uint64_t state_;
};

}  // namespace rl
}  // namespace simul

#endif  // CORE_CRL_RANDOM_H
