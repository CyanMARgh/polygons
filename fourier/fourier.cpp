#include "fourier.h"
#include <cmath>

c16 one_cpow(float t) {
	return {cosf(t), sinf(t)};
}
std::vector<c16> DFT(const std::vector<c16>& data, u32 order) {
	u32 N = 1 << order, n = data.size();
	float eps = 2.f * M_PI / N;
	std::vector<c16> result(n);
	for(u32 i = 0; i < n; i++) {
		c16 sum = {0.f, 0.f};
		for(u32 j = 0; j < n; j++) {
			sum += one_cpow(eps * ((i * j) % N)) * data[j];
		}
		result[i] = sum;
	}
	return result;
}
std::vector<c16> inv_DFT(const std::vector<c16>& data, u32 order) {
	u32 N = 1 << order, n = data.size();
	float eps = -2.f * M_PI / N;
	std::vector<c16> result(n);
	for(u32 i = 0; i < n; i++) {
		c16 sum = {0.f, 0.f};
		for(u32 j = 0; j < n; j++) {
			sum += one_cpow(eps * ((i * j) % N)) * data[j];
		}
		result[i] = sum / (float)n;
	}
	return result;
}
void print(const std::vector<c16>& data) {
	putchar('[');
	for(u32 i = 0, n = data.size(); i < n; i++) {
		c16 c = data[i];
		printf("%5.2f+%5.2fi%s", c.real(), c.imag(), i == n - 1 ? "]\n" : ", ");
	}
}

std::vector<c16> FFT(const std::vector<c16>& data, u32 order) {
	if(order) {
		u32 n = data.size(), n2 = n / 2;
		c16 w = one_cpow(2.f * M_PI / n);
		std::vector<c16> even(n2), odd(n2);
		for(u32 i = 0; i < n2; i++) {
			even[i] = data[2 * i];
			odd[i] = data[2 * i + 1];
		}
		std::vector<c16> fodd = FFT(odd, order - 1), feven = FFT(even, order - 1);
		std::vector<c16> ftotal(n);
		for(u32 i = 0; i < n2; i++) {
			c16 m = pow(w, (float)i);
			ftotal[i] = feven[i] + m * fodd[i];
			ftotal[i + n2] = feven[i] - m * fodd[i];
		}
		return ftotal;
	} else {
		return data;
	}
}
std::vector<c16> inv_FFT_iter(const std::vector<c16>& data, u32 order) {
	if(order) {
		u32 n = data.size(), n2 = n / 2;
		c16 w = one_cpow(-2.f * M_PI / n);
		std::vector<c16> even(n2), odd(n2);
		for(u32 i = 0; i < n2; i++) {
			even[i] = data[2 * i];
			odd[i] = data[2 * i + 1];
		}
		std::vector<c16> fodd = inv_FFT_iter(odd, order - 1), feven = inv_FFT_iter(even, order - 1);
		std::vector<c16> ftotal(n);
		for(u32 i = 0; i < n2; i++) {
			c16 m = pow(w, (float)i);
			ftotal[i] = feven[i] + m * fodd[i];
			ftotal[i + n2] = feven[i] - m * fodd[i];
		}
		return ftotal;
	} else {
		return data;
	}
}
std::vector<c16> inv_FFT(const std::vector<c16>& data, u32 order) {
	std::vector<c16> result = FFT(data, order);
	float n = data.size();
	for(c16& x : result) x /= n;
	return result;
}

