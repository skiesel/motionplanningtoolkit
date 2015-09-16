#pragma once
#include <functional>
#include <vector>
#include <cassert>

template <class T, class Ops> class RBTree {
	enum COLOR { RED, BLACK };

	struct RBNode {
		RBNode() : key(NULL), parent(this), left(this), right(this), color(BLACK) {}
		RBNode(T *key, RBNode* nil) : key(key), parent(nil), left(nil), right(nil), color(RED) {}
		~RBNode() {
			//explicitly NOT deleting key
		}
		T *key;
		RBNode *parent, *left, *right;
		COLOR color;
	};

public:

	RBTree() {
		root = nil = new RBNode(); 
	}

	~RBTree() {
		if(root == nil) return;

		std::vector<RBNode*> tree;

		tree.push_back(root);
		while(!tree.empty()) {
			RBNode* n = tree.back();
			tree.pop_back();
			if(n->left != nil) {
				tree.push_back(n->left);
			}
			if(n->right != nil) {
				tree.push_back(n->right);
			}
			delete n;
		}
	}

	bool isEmpty() const {
		return root == nil;
	}

	const T* peekLeftmost() const {
		if(root == nil) return NULL;

		RBNode* min = root;
		while(min->left != nil) {
			min = min->left;
		}
		return min->key;
	}

	T* extractLeftmost() {
		if(root == nil) return NULL;

		RBNode* min = root;
		while(min->left != nil) {
			min = min->left;
		}
		T* key = min->key;

		//already have the RBNode, so delete using that
		remove(min);

		return key;
	}

	std::vector<T*> getContiguousRange(std::function<bool(T*)> includeThis) {
		std::vector<T*> range;

		if(root == nil) return range;

		int collecting = 0;
		
		getContiguousRange(root, includeThis, range, &collecting);

		return range;
	}

	void insert(T *elem) {
		insert(new RBNode(elem, nil));
	}

	void remove(T *elem) {
		RBNode *z = root;
		while(z != nil) {
			int comparison = Ops::compare(elem, z->key);
			if(comparison < 0) {
					z = z->left;
			} else if(comparison > 0) {
				z = z->right;
			} else {
				remove(z);
				return;
			}
		}
	}

	void checkInvariants() const {
		assert(root->color == BLACK);
		
		int first_black_height = -1;

		checkInvariants(root, 1, &first_black_height);
	}

private:

	void getContiguousRange(RBNode* node, std::function<bool(T*)> includeThis, std::vector<T*>& range, int* collecting) {
		
		if(node->left != nil) {
			getContiguousRange(node->left, includeThis, range, collecting);
			if(*collecting == 2) {
				return;
			}
		}

		if(includeThis(node->key)) {
			*collecting = 1;
			range.push_back(node->key);
		} else if(*collecting == 1) {
			*collecting = 2;
		}

		if(node->right != nil) {
			getContiguousRange(node->right, includeThis, range, collecting);
			if(*collecting == 2) {
				return;
			}
		}
	}

	void checkInvariants(RBNode* node, unsigned int black_nodes, int* first_black_height) const {
		if(node->left == nil && node->right == nil) {
			if(*first_black_height < 0) {
				*first_black_height = black_nodes;
			} else {
				assert(black_nodes == *first_black_height);
			}
		}

		if(node->left != nil) {
			assert(Ops::compare(node->key, node->left->key) >= 0);
			assert(node->color == BLACK || node->left->color == BLACK);
			
			int new_black = black_nodes + (node->left->color == BLACK ? 1 : 0);
			
			checkInvariants(node->left, new_black, first_black_height);
		}
		if(node->right != nil) {
			assert(Ops::compare(node->key, node->right->key) <= 0);
			assert(node->color == BLACK || node->right->color == BLACK);
			
			int new_black = black_nodes + (node->right->color == BLACK ? 1 : 0);

			checkInvariants(node->right, new_black, first_black_height);
		}
	}

	void insert(RBNode *z) {
		RBNode *x = root;
		RBNode *y = nil;
		while(x != nil) {
			y = x;
			if(Ops::compare(z->key, x->key) <= 0) {
				x = x->left;
			} else {
				x = x->right;
			}
		}

		z->parent = y;

		if(y == nil) {
			root = z;
		} else if(Ops::compare(z->key, y->key) <= 0) {
			y->left = z;
		} else {
			y->right = z;
		}

		insertFixup(z);
	}

	void insertFixup(RBNode *z) {
		RBNode *y = nil;

		while(z->parent->color == RED) {
			if(z->parent == z->parent->parent->left) {
				y = z->parent->parent->right;
				if(y->color == RED) {
					z->parent->color = BLACK;
					y->color = BLACK;
					z->parent->parent->color = RED;
					z = z->parent->parent;
				} else {
					if(z == z->parent->right) {
						z = z->parent;
						rotateLeft(z);
					}
					z->parent->color = BLACK;
					z->parent->parent->color = RED;
					rotateRight(z->parent->parent);
				}
			} else {
				y = z->parent->parent->left;
				if(y->color == RED) {
					z->parent->color = BLACK;
					y->color = BLACK;
					z->parent->parent->color = RED;
					z = z->parent->parent;
				} else {
					if(z == z->parent->left) {
						z = z->parent;
						rotateRight(z);
					}
					z->parent->color = BLACK;
					z->parent->parent->color = RED;
					rotateLeft(z->parent->parent);
				}
			}
		}
		root->color = BLACK;
	}

	void rotateLeft(RBNode *x) {
		RBNode *y = x->right;
		
		x->right = y->left;
		if(y->left != nil) {
			y->left->parent = x;
		}
		
		y->parent = x->parent;

		if(x->parent == nil) {
			root = y;
		} else if(x == x->parent->left) {
			x->parent->left = y;
		} else {
			x->parent->right = y;
		}

		y->left = x;
		x->parent = y;
	}

	void rotateRight(RBNode *x) {
		RBNode *y = x->left;
		
		x->left = y->right;
		if(y->right != nil) {
			y->right->parent = x;
		}
		
		y->parent = x->parent;

		if(x->parent == nil) {
			root = y;
		} else if(x == x->parent->left) {
			x->parent->left = y;
		} else {
			x->parent->right = y;
		}

		y->right = x;
		x->parent = y;
	}	

	void remove(RBNode *z) {
		RBNode *x = nil;
		RBNode *y = z;
		COLOR y_original_color = y->color;

		if(z->left == nil) {
			x = z->right;
			transplant(z, z->right);
		} else if(z->right == nil) {
			x = z->left;
			transplant(z, z->left);
		} else {
			y = successor(z);
			y_original_color = y->color;
			x = y->right;
			if(y->parent == z) {
				x->parent = y;
			} else {
				transplant(y, y->right);
				y->right = z->right;
				y->right->parent = y;
			}
			transplant(z, y);
			y->left = z->left;
			y->left->parent = y;
			y->color = z->color;
		}
		if(y_original_color == BLACK) {
			removeFixup(x);
		}
	}

	void removeFixup(RBNode* x) {
		RBNode* w = nil;

		while(x != root && x->color == BLACK) {
			if(x == x->parent->left) {
				w = x->parent->right;
				if(w->color == RED) {
					w->color = BLACK;
					x->parent->color = RED;
					rotateLeft(x->parent);
					w = x->parent->right;
				}
				if(w->left->color == BLACK && w->right->color == BLACK) {
					w->color = RED;
					x = x->parent;
				} else {
					if(w->right->color == BLACK) {
						w->left->color = BLACK;
						w->color = RED;
						rotateRight(w);
						w = x->parent->right;
					}
					w->color = x->parent->color;
					x->parent->color = BLACK;
					w->right->color = BLACK;
					rotateLeft(x->parent);
					x = root;
				}
			} else {
				w = x->parent->left;
				if(w->color == RED) {
					w->color = BLACK;
					x->parent->color = RED;
					rotateRight(x->parent);
					w = x->parent->left;
				}
				if(w->left->color == BLACK && w->right->color == BLACK) {
					w->color = RED;
					x = x->parent;
				} else {
					if(w->left->color == BLACK) {
						w->right->color = BLACK;
						w->color = RED;
						rotateLeft(w);
						w = x->parent->left;
					}
					w->color = x->parent->color;
					x->parent->color = BLACK;
					w->left->color = BLACK;
					rotateRight(x->parent);
					x = root;
				}
			}
		}
		x->color = BLACK;
	}

	void transplant(RBNode *u, RBNode *v) {
		if(u->parent == nil) {
			root = v;
		} else if(u == u->parent->left) {
			u->parent->left = v;
		} else {
			u->parent->right = v;
		}
		v->parent = u->parent;
	}

	RBNode* successor(const RBNode* x) const {
		if(x->right == nil) return NULL;
		RBNode* min = x->right;
		while(min->left != nil) {
			min = min->left;
		}
		return min;
	}

	RBNode* root;
	RBNode* nil;
};