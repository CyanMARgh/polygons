#include "tree.h"
#include "utils.h"
#include <iostream>
#include <cmath>

u32 Tree::Node::Heigth(const Tree::Node* node) {
	return node ? node->heigth : 0;
}
void Tree::Node::ConnectChilds(Tree::Node* node) {
	if(!node) return;
	if(node->left) { 
		node->left->parent = node;
	}
	if(node->right) {
		node->right->parent = node;
	}
}
void Tree::Node::UpdateHeigth(Tree::Node* node) {
	if(!node) return;
	u32 hl = Heigth(node->left) + 1, hr = Heigth(node->right) + 1;
	node->heigth = hl > hr ? hl : hr;
}
Tree::Node::Node(Key key, Val val) {
	this->key = key, this->val = val;
	heigth = 1, left = nullptr, right = nullptr;
}
Tree::Node* Tree::Node::Copy(Tree::Node* node) {
	if(!node) return nullptr;
	Node* copy = new Node(node->key, node->val);
	copy->heigth = node->heigth;
	copy->left = Copy(node->left), copy->right = Copy(node->right);
	ConnectChilds(copy);
	return copy;
}
Tree::Node* Tree::Node::Balance(Tree::Node* node) {
	s32 hl = Heigth(node->left), hr = Heigth(node->right), d = hr - hl;
	if(d > 1) {
		Tree::Node *R = node->right, *A = node->left, *B = R->left, *C = R->right;
		R->left = node;
		node->right = B;
		ConnectChilds(node), ConnectChilds(R);
		UpdateHeigth(node), UpdateHeigth(R);
		return R;
	} else if (d < -1) {
		Tree::Node *L = node->left, *A = node->right, *B = L->right, *C = L->left;
		L->right = node;
		node->left = B;
		ConnectChilds(node), ConnectChilds(L);
		UpdateHeigth(node), UpdateHeigth(L);
		return L;
	} else {
		return node;
	}
}

Tree::Node* Tree::Node::Add(Tree::Node* node, Key key, Val val) {
	if(!node) return new Node(key, val);
	Key nkey = node->key;
	if(nkey > key) {
		node->left = Add(node->left, key, val);
	} else {
		node->right = Add(node->right, key, val);
	}
	ConnectChilds(node);
	UpdateHeigth(node);
	return Balance(node);
}
Tree::Node* Tree::Node::Remove(Tree::Node* node, Key key) {
	if(!node) return nullptr;
	Key nkey = node->key;
	if(nkey > key) {
		node->left = Remove(node->left, key);
	} else if (nkey < key) {
		node->right = Remove(node->right, key);
	} else {
		delete node;
		return nullptr;
	}
	ConnectChilds(node);
	UpdateHeigth(node);
	return Balance(node);	
}
Val* Tree::Node::At(Tree::Node* node, Key key) {
	if(node) {
		Key nkey = node->key;
		if(nkey > key) {
			return At(node->left, key);
		} else if(nkey < key) {
			return At(node->right, key);
		} else {
			return &(node->val);
		}
	} else {
		return nullptr;
	}
}
void Tree::Node::Print(const Tree::Node* node, u32 depth) {
	if(!node) return;
	Print(node->right, depth + 1);
	for(u32 i = 0; i < depth; i++)printf("  ");
	std::cout << "[" << node->key << ": " << node->val << "]\n";
	Print(node->left, depth + 1);
}
void Tree::Node::Clear(Node* node) {
	if(!node) return;
	Clear(node->left);
	Clear(node->right);
	delete node;
}

Tree::Tree() {
	root = nullptr;
}
Tree::Tree(const Tree& other) {
	root = Node::Copy(other.root);
}
Tree& Tree::operator=(const Tree& other) {
	if(&other != this) {
		Node::Clear(root);
		root = Node::Copy(other.root);
	}
	return *this;
}
Tree::~Tree() {
	Node::Clear(root);
}
void Tree::Add(Key key, Val val) {
	root = Node::Add(root, key, val);
}
Val* Tree::At(Key key) {
	return Node::At(root, key);
}
void Tree::Remove(Key key) {
	root = Node::Remove(root, key);
}
void Tree::Print() const {
	Node::Print(root, 0);
}
