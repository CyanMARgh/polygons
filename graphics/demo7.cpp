#include "demos.h"
#include "fourier.h"
#include "utils.h"

void demo::fourier() {
	u32 size = 8, order = 3;
	std::vector<c16> data(size);
	for(u32 i = 0; i < size; i++) {
		data[i] = randf();
		//data[i] = i + 1;
	}
	auto data1 = FFT(data, order);
	auto data2 = inv_FFT(data1, order);
	//auto data3 = DFT(data, order);
	print(data);
	print(data2);
}