#include "demos.h"
#include "tree.h"

void demo::Trees() {
	Tree t;
	for(u32 i = 0; i < 20; i++) {
		u32 a = rand() % 30, b = rand() % 100;
		//printf("t[%u] = %u\n", a, b);
		t.Add(a, b);
	}
	t.Print();
}