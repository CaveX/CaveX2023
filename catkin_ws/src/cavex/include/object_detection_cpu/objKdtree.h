#pragma once

#include "objRender.h"

struct Node {
    std::vector<float> point;
    int id;
    Node *left;
    Node *right;

    Node(std::vector<float> arr, int setId) : point(arr), id(setId), left(NULL), right(NULL) {}
};

struct KdTree {
    Node *rootNode;

    KdTree() : rootNode(NULL) {}

    void insertHelper(Node **node, unsigned int depth, std::vector<float> point, int id) {
        if(*node == NULL) *node = new Node(point, id);
        else {
            unsigned int cd = depth % 3;

            if(point[cd] < ((*node)->point[cd])) insertHelper(&((*node)->left), depth + 1, point, id);
            else insertHelper(&((*node)->right), depth + 1, point, id);
        }
    }
    
    void insert(std::vector<float> point, int id) {
        insertHelper(&rootNode, 0, point, id);
    }

    void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTolerance, std::vector<int> &ids) {
        if(node != NULL) {
            if((node->point[0] >= (target[0] - distanceTolerance) && node->point[0] <= (target[0] + distanceTolerance)) && (node->point[1] >= (target[1] - distanceTolerance) && node->point[1] <= (target[1] + distanceTolerance))) {
                float distance = sqrt((node->point[0] - target[0])*(node->point[0] - target[0]) + (node->point[1] - target[1])*(node->point[1] - target[1]));
                if(distance <= distanceTolerance) ids.push_back(node->id);
            }
            if((target[depth % 3] - distanceTolerance) < node->point[depth % 3]) {
                searchHelper(target, node->left, depth + 1, distanceTolerance, ids);
            }
            if((target[depth % 3] + distanceTolerance) > node->point[depth % 3]) {
                searchHelper(target, node->right, depth + 1, distanceTolerance, ids);
            }
        }
    }

    void searchHelper3D(std::vector<float> target, Node *node, int depth, float distanceTolerance, std::vector<int> &ids) {
        if(node != NULL) {
            if((node->point[0] >= (target[0] - distanceTolerance) && (node->point[0] >= (target[0] + distanceTolerance))
            && (node->point[1] >= (target[1] - distanceTolerance)) && (node->point[1] >= (target[1] + distanceTolerance))
            && (node->point[2] >= (target[2] - distanceTolerance)) && (node->point[2] >= (target[2] + distanceTolerance)))) {
                float distance = sqrt((node->point[0] - target[0])*(node->point[0] - target[0]) + (node->point[1] - target[1])*(node->point[1] - target[1]) + (node->point[2] - target[2])*(node->point[2] - target[2]));
                if(distance <= distanceTolerance) ids.push_back(node->id);
                if((target[depth % 3] - distanceTolerance) < node->point[depth % 3]) searchHelper3D(target, node->left, depth + 1, distanceTolerance, ids);
                if((target[depth % 3] + distanceTolerance) > node->point[depth % 3]) searchHelper3D(target, node->right, depth + 1, distanceTolerance, ids);
            }
        }
    }

    std::vector<int> search(std::vector<float> target, float distanceTolerance) {
        std::vector<int> ids;
        searchHelper(target, rootNode, 0, distanceTolerance, ids);
        return ids;
    }
};