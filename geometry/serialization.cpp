#include "polygon.h"


geom::invalid_read::invalid_read(const std::string& filename): std::invalid_argument("invalid file or data on deserialization: " + filename) { }

std::ofstream& operator<<(std::ofstream& fout, const poly& P) {
	fout.write((const char*)&P.size, sizeof(u32));
	fout.write((const char*)&(P.points[0]), P.size * sizeof(vec2));
	return fout;
}
std::ifstream& operator>>(std::ifstream& fin, poly& P) {
	fin.read((char*)&P.size, sizeof(u32));
	P.points.resize(P.size);
	fin.read((char*)&(P.points[0]), P.size * sizeof(vec2));
	return fin;
}
