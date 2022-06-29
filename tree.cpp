#include "tree.h"
#include "utils.h"
#include <iostream>
#include <cmath>

u32 tree::node::get_heigth(const tree::node* n) {
	return n ? n->heigth : 0;
}
void tree::node::connect_childs(tree::node* n) {
	if(!n) return;
	if(n->left) { 
		n->left->parent = n;
	}
	if(n->right) {
		n->right->parent = n;
	}
}
void tree::node::update_heigth(tree::node* n) {
	if(!n) return;
	u32 hl = get_heigth(n->left) + 1, hr = get_heigth(n->right) + 1;
	n->heigth = hl > hr ? hl : hr;
}
tree::node::node(key k, val v) {
	this->k = k, this->v = v;
	heigth = 1, left = nullptr, right = nullptr;
}
tree::node* tree::node::copy(tree::node* n) {
	if(!n) return nullptr;
	node* nc = new node(n->k, n->v);
	nc->heigth = n->heigth;
	nc->left = copy(n->left), nc->right = copy(n->right);
	connect_childs(nc);
	return nc;
}
tree::node* tree::node::balance(tree::node* n) {
	s32 hl = get_heigth(n->left), hr = get_heigth(n->right), d = hr - hl;
	if(d > 1) {
		tree::node *R = n->right, *A = n->left, *B = R->left, *C = R->right;
		R->left = n;
		n->right = B;
		connect_childs(n), connect_childs(R);
		update_heigth(n), update_heigth(R);
		return R;
	} else if (d < -1) {
		tree::node *L = n->left, *A = n->right, *B = L->right, *C = L->left;
		L->right = n;
		n->left = B;
		connect_childs(n), connect_childs(L);
		update_heigth(n), update_heigth(L);
		return L;
	} else {
		return n;
	}
}

tree::node* tree::node::add(tree::node* n, key k, val v) {
	if(!n) return new node(k, v);
	key nk = n->k;
	if(nk > k) {
		n->left = add(n->left, k, v);
	} else {
		n->right = add(n->right, k, v);
	}
	connect_childs(n);
	update_heigth(n);
	return balance(n);
}
tree::node* tree::node::remove(tree::node* n, key k) {
	if(!n) return nullptr;
	key nk = n->k;
	if(nk > k) {
		n->left = remove(n->left, k);
	} else if (nk < k) {
		n->right = remove(n->right, k);
	} else {
		delete n;
		return nullptr;
	}
	connect_childs(n);
	update_heigth(n);
	return balance(n);	
}
val* tree::node::at(tree::node* n, key k) {
	if(n) {
		key nk = n->k;
		if(nk > k) {
			return at(n->left, k);
		} else if(nk < k) {
			return at(n->right, k);
		} else {
			return &(n->v);
		}
	} else {
		return nullptr;
	}
}
void tree::node::print(const tree::node* n, u32 depth) {
	if(!n) return;
	print(n->right, depth + 1);
	for(u32 i = 0; i < depth; i++)printf("  ");
	std::cout << "[" << n->k << ": " << n->v << "]\n";
	print(n->left, depth + 1);
}
void tree::node::clear(node* n) {
	if(!n) return;
	clear(n->left);
	clear(n->right);
	delete n;
}

tree::tree() {
	root = nullptr;
}
tree::tree(const tree& other) {
	root = node::copy(other.root);
}
tree& tree::operator=(const tree& other) {
	if(&other != this) {
		node::clear(root);
		root = node::copy(other.root);
	}
	return *this;
}
tree::~tree() {
	node::clear(root);
}
void tree::add(key k, val v) {
	root = node::add(root, k, v);
}
val* tree::at(key k) {
	return node::at(root, k);
}
void tree::remove(key k) {
	root = node::remove(root, k);
}
void tree::print() const {
	node::print(root, 0);
}
