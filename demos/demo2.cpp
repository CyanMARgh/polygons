#include "demos.h"
#include "tree.h"

void demo::trees() {
	tree t;
	for(u32 i = 0; i < 20; i++) {
		u32 a = rand() % 30, b = rand() % 100;
		//printf("t[%u] = %u\n", a, b);
		t.add(a, b);
	}
	t.print();
}