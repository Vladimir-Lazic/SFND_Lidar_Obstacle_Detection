#ifndef KDTREE_H
#define KDTREE_H

#include <iostream>
#include <vector>
#include <string>
#include <math.h>

struct Node
{
    std::vector<float> point;
    int id;
    Node *left;
    Node *right;

    Node(std::vector<float> arr, int setId)
        : point(arr), id(setId), left(NULL), right(NULL)
    {
    }
};

struct KdTree
{
    Node *root;

    KdTree()
        : root(NULL)
    {
    }

    void instert_node(Node **node, int depth, std::vector<float> point, int id)
    {
        if (*node == NULL)
        {
            *node = new Node(point, id);
        }
        else
        {
            int split = depth % 3;

            if (point[split] < (*node)->point[split])
            {
                instert_node(&(*node)->left, ++depth, point, id);
            }
            else
            {
                instert_node(&(*node)->right, ++depth, point, id);
            }
        }
    }

    void insert(std::vector<float> point, int id)
    {
        instert_node(&root, 0, point, id);
    }

    void search_helper(std::vector<float> target, Node *node, int depth, float distanceTol, std::vector<int> &ids)
    {
        if (node != NULL)
        {
            if ((node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol)) && (node->point[1] >= (target[1] - distanceTol) && node->point[1] <= (target[1] + distanceTol)) &&
                (node->point[2] >= (target[2] - distanceTol) && node->point[2] <= (target[2] + distanceTol)))
            {
                float distance = sqrt(pow((node->point[0] - target[0]), 2) + pow((node->point[1] - target[1]), 2) + pow((node->point[2] - target[2]), 2));
                if (distance <= distanceTol)
                {
                    ids.push_back(node->id);
                }
            }

            if (target[depth % 3] - distanceTol < node->point[depth % 3])
            {
                search_helper(target, node->left, ++depth, distanceTol, ids);
            }

            if (target[depth % 3] + distanceTol > node->point[depth % 3])
            {
                search_helper(target, node->right, ++depth, distanceTol, ids);
            }
        }
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int>
    search(std::vector<float> target, float distanceTol)
    {
        std::vector<int> ids;
        search_helper(target, root, 0, distanceTol, ids);
        return ids;
    }
};

#endif // KDTREE_H