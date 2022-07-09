#pragma once

#include "utils.h"

typedef s32 key;
typedef s32 val;

class tree {
	public:
	struct node {
		u32 heigth;
		node *parent, *left, *right;
		key k;
		val v;

		node(key k, val v);

		static node* copy(node* n);
		static void update_heigth(node* n);
		static void connect_childs(node* n);
		static node* add(node* n, key k, val v);
		static node* remove(node* n, key k);
		static val* at(node* n, key k);
		static node* balance(node* n);
		static void print(const node* n, u32 depth);
		static void clear(node* n);
		static u32 get_heigth(const node* n);
	};
	private:
	node* root;
	public:
	tree();
	tree(const tree& other);
	//Tree(Tree&& other);
	tree& operator=(const tree& other);
	//Tree& operator=(Tree&& other);
	~tree();
	void add(key k, val v);
	val* at(key k);
	void remove(key k);
	void print() const;
};