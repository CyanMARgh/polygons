#include <complex>
#include <vector>
#include "utils.h"

typedef std::complex<float> c16;

c16 one_cpow(float t);
std::vector<c16> DFT(const std::vector<c16>& data, u32 order);
std::vector<c16> inv_DFT(const std::vector<c16>& data, u32 order);

std::vector<c16> FFT(const std::vector<c16>& data, u32 order);
std::vector<c16> inv_FFT(const std::vector<c16>& data, u32 order);

void print(const std::vector<c16>& data);
