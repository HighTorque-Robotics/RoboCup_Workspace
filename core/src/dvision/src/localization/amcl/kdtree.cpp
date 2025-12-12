/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * @file kdtree.cpp
 * @author Yusu Pan, Wenxing Mei
 * @version 2018
 * @date 2018-02-23
 */

#include "dvision/amcl/kdtree.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>

namespace dvision {
KdTree::KdTree(size_t max_size)
{
    size_[0] = 10;
    size_[1] = 10;
    size_[2] = (30.0 * M_PI / 180);

    root_ = NULL;
    leaf_count_ = 0;
    node_count_ = 0;
    node_max_count_ = max_size;
    nodes_.resize(max_size);
    for (auto& node : nodes_) {
        node = new KdTreeNode();
        node->children[0] = nullptr;
        node->children[1] = nullptr;
    }
}

KdTree::~KdTree()
{
    Clear();
}

void
KdTree::Clear()
{
    root_ = NULL;
    leaf_count_ = 0;
    node_count_ = 0;
}

void
KdTree::InsertPose(Pose pose, double weight)
{
    int key[3];

    key[0] = static_cast<int>(floor(pose.x() / size_[0]));
    key[1] = static_cast<int>(floor(pose.y() / size_[1]));
    key[2] = static_cast<int>(floor(pose.headingR() / size_[2]));

    root_ = InsertNode(NULL, root_, key, weight);

    // Test
    assert(FindNode(root_, key) != NULL);

    for (size_t i = 0; i < node_count_; ++i) {
        auto& node = nodes_[i];
        if (node->leaf) {
            assert(FindNode(root_, node->key) == node);
        }
    }
}

KdTreeNode*
KdTree::InsertNode(KdTreeNode* parent, KdTreeNode* node, int key[], double value)
{
    if (node == NULL) {
        assert(node_count_ < node_max_count_);
        node = nodes_[node_count_++];
        memset(node, 0, sizeof(KdTreeNode));

        node->leaf = true;
        if (parent == NULL)
            node->depth = 0;
        else
            node->depth = parent->depth + 1;

        for (int i = 0; i < 3; ++i)
            node->key[i] = key[i];

        node->value = value;
        ++leaf_count_;
    }

    // If the node exists, and it is a leaf node ...
    else if (node->leaf) {
        // If the keys are equal, increment the value
        if (node->keyEqual(key)) {
            node->value += value;
        }
        // The keys are not equal, so split this node
        else {
            // Find the dimension with the largest variance and do a mean split
            int max_split = 0;
            int split;
            node->pivot_dim = -1;
            for (int i = 0; i < 3; ++i) {
                split = abs(key[i] - node->key[i]);
                if (split > max_split) {
                    max_split = split;
                    node->pivot_dim = i;
                }
            }
            assert(node->pivot_dim >= 0);

            node->pivot_value = (key[node->pivot_dim] + node->key[node->pivot_dim]) / 2.0;

            if (key[node->pivot_dim] < node->pivot_value) {
                node->children[0] = InsertNode(node, NULL, key, value);
                node->children[1] = InsertNode(node, NULL, node->key, node->value);
            } else {
                node->children[0] = InsertNode(node, NULL, node->key, node->value);
                node->children[1] = InsertNode(node, NULL, key, value);
            }

            node->leaf = false;
            --leaf_count_;
        }
    }

    // If the node exists, and it has children ...
    else {
        assert(node->children[0] != NULL);
        assert(node->children[1] != NULL);

        if (key[node->pivot_dim] < node->pivot_value) {
            InsertNode(node, node->children[0], key, value);
        } else {
            InsertNode(node, node->children[1], key, value);
        }
    }
    return node;
}

KdTreeNode*
KdTree::FindNode(KdTreeNode* node, int key[])
{
    if (node->leaf) {
        if (node->keyEqual(key)) {
            return node;
        } else {
            return NULL;
        }
    } else {
        assert(node->children[0] != NULL);
        assert(node->children[1] != NULL);
    }

    // If the keys are different
    if (key[node->pivot_dim] < node->pivot_value)
        return FindNode(node->children[0], key);
    else
        return FindNode(node->children[1], key);
}

void
KdTree::Cluster()
{
    std::vector<KdTreeNode*> queue;

    // Put all the leaves in a queue
    for (size_t i = 0; i < node_count_; ++i) {
        KdTreeNode* node = nodes_[i];
        if (node->leaf) {
            node->cluster = -1;
            queue.push_back(node);
            assert(node == FindNode(root_, node->key));
        }
    }
    int cluster_count = 0;
    // Do connected components for each node
    for (int i = queue.size() - 1; i >= 0; --i) {
        KdTreeNode* node = queue[i];
        assert(node != NULL);
        // If this node has already been labelled, skip it
        if (node->cluster >= 0)
            continue;

        // Assign a label to this cluster
        node->cluster = cluster_count++;
        ClusterNode(node, 0);
    }
}

// D ... DFS ???
void
KdTree::ClusterNode(KdTreeNode* node, int depth)
{
    int nkey[3];
    for (int i = 0; i < 3 * 3 * 3; ++i) {
        nkey[0] = node->key[0] + (i / 9) - 1;
        nkey[1] = node->key[1] + ((i % 9) / 3) - 1;
        nkey[2] = node->key[2] + ((i % 9) % 3) - 1;

        KdTreeNode* nnode = FindNode(root_, nkey);
        if (nnode == NULL)
            continue;
        else if (nnode == node)
            continue;

        assert(nnode->leaf);

        // This node already has a leabel; skip it. The label should be consistent, however
        if (nnode->cluster >= 0) {
            assert(nnode->cluster == node->cluster);
            continue;
        }

        nnode->cluster = node->cluster;

        // Label this node and recurse
        ClusterNode(nnode, depth + 1);
    }
}

int
KdTree::GetCluster(Pose pose)
{

    int key[3];

    key[0] = static_cast<int>(floor(pose.x() / size_[0]));
    key[1] = static_cast<int>(floor(pose.y() / size_[1]));
    key[2] = static_cast<int>(floor(pose.headingR() / size_[2]));

    auto node = FindNode(root_, key);
    if (node == NULL) {
        return -1;
    }
    return node->cluster;
}

void
KdTree::PrintNode(KdTreeNode* node)
{
    if (node->leaf) {
        printf("(%+02d %+02d %+02d)\n", node->key[0], node->key[1], node->key[2]);
        printf("%*s", node->depth * 11, "");
    } else {
        printf("(%+02d %+02d %+02d) ", node->key[0], node->key[1], node->key[2]);
        PrintNode(node->children[0]);
        PrintNode(node->children[1]);
    }
}
}
