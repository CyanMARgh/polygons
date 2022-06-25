#pragma once

#include "utils.h"

typedef s32 Key;
typedef s32 Val;

class Tree {
	public:
	struct Node {
		u32 heigth;
		Node *parent, *left, *right;
		Key key;
		Val val;

		Node(Key key, Val value);

		static Node* Copy(Node* node);
		static void UpdateHeigth(Node* node);
		static void ConnectChilds(Node* node);
		static Node* Add(Node* node, Key key, Val val);
		static Node* Remove(Node* node, Key key);
		static Val* At(Node* node, Key key);
		static Node* Balance(Node* node);
		static void Print(const Node* node, u32 depth);
		static void Clear(Node* node);
		static u32 Heigth(const Node* node);
	};
	private:
	Node* root;
	public:
	Tree();
	Tree(const Tree& other);
	//Tree(Tree&& other);
	Tree& operator=(const Tree& other);
	//Tree& operator=(Tree&& other);
	~Tree();
	void Add(Key key, Val val);
	Val* At(Key key);
	void Remove(Key key);
	void Print() const;
};