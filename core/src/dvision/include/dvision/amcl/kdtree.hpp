/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * @file kdtree.hpp
 * @author Yusu Pan, Wenxing Mei
 * @version 2018
 * @date 2018-02-23
 */

#pragma once
#include "dvision/amcl/pose.hpp"
#include <array>
#include <vector>

namespace dvision {
//! Infomation for a node in the tree
struct KdTreeNode
{
    /**
     * @brief KdTreeNode constructor
     */
    KdTreeNode()
      : leaf(true)
    {
    }

    bool operator==(const KdTreeNode& oth) const
    {
        return (key[0] == oth.key[0]) && (key[1] == oth.key[1]) && (key[2] == oth.key[2]);
    }

    /**
     * @brief Check whether or not node key is equal to given key
     *
     * @param othKey - given key to check
     *
     * @return whether or not node key is equal to given key
     */
    bool keyEqual(int othKey[])
    {
        return (key[0] == othKey[0]) && (key[1] == othKey[1]) && (key[2] == othKey[2]);
    }

    //! Whether or not this node is a Leaf node
    bool leaf;

    //! Depth in the tree
    int depth;

    //! Pivot and value
    int pivot_dim;
    //! Pivot  value
    double pivot_value;

    //! The key for this node
    int key[3];

    //! The value for this node
    double value;

    //! The cluster label (leaf nodes)
    int cluster;

    //! Child nodes
    std::array<KdTreeNode*, 2> children;
};

//! A kd tree
class KdTree
{
  public:
    /**
     * @brief KdTree constructor
     *
     * @param max_size - maximum size of kd tree
     */
    explicit KdTree(size_t max_size);

    /**
     * @brief KdTree destructor
     */
    ~KdTree();

    /**
     * @brief Clear kd tree
     */
    void Clear();

    /**
     * @brief Insert a pose into the tree
     *
     * @param pose - pose to insert
     * @param weight - weight of pose
     */
    void InsertPose(Pose pose, double weight);

    /**
     * @brief Insert a node into the tree
     *
     * @param parent - parent node
     * @param node - node to insert
     * @param key - key of node
     * @param value - value of node
     *
     * @return inserted node
     */
    KdTreeNode* InsertNode(KdTreeNode* parent, KdTreeNode* node, int key[], double value);

    /**
     * @brief Find node with key
     *
     * @param node - given node to find
     * @param key - key of node
     *
     * @return found node
     */
    KdTreeNode* FindNode(KdTreeNode* node, int key[]);

    /**
     * @brief Cluster the leaves in the tree
     */
    void Cluster();

    /**
     * @brief Recursively label nodes in this cluster
     *
     * @param node - node to label
     * @param depth - given depth
     */
    void ClusterNode(KdTreeNode* node, int depth);

    /**
     * @brief Determine the cluster label for the given pose
     *
     * @param p - given pose
     *
     * @return corresponding depth for given pose
     */
    int GetCluster(Pose p);

    /**
     * @brief Print node
     *
     * @param node - node to print
     */
    void PrintNode(KdTreeNode* node);

    //! Cell size
    double size_[3];

    //! The root node of the tree
    KdTreeNode* root_;
    //! Nodes of the tree
    std::vector<KdTreeNode*> nodes_;

    //! The number of nodes in the tree
    size_t node_count_;
    //! Maximum number of nodes in the tree
    size_t node_max_count_;

    //! The number of leaf nodes in the tree
    int leaf_count_;
};
}
