#include <vector>
#include <cmath>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

struct Node
{
    pcl::PointXYZI point; 
    int id;
    Node* left;
    Node* right;

    Node(pcl::PointXYZI arr, int setId)
    {
        this->point = arr;
        this->id = setId;
        left = nullptr;
        right = nullptr;
    }
};

struct KDTree
{
    Node* root;

    KDTree()
    {
        this->root = nullptr;
    }

    void insertCore(Node** node, uint depth, pcl::PointXYZI point, int id)
    {
        if(*node==NULL) 
        {
            *node = new Node(point, id);
        }
        else
        {
            if(depth % 3 == 0)
            {
                if(point.x < ((*node)->point.x)) 
                {
                    insertCore(&((*node)->left), depth+1, point, id);
                }
                else 
                {
                    insertCore(&((*node)->right), depth+1, point, id);    
                }
            }
            else if(depth % 3 == 1)
            {
                if(point.y < ((*node)->point.y)) 
                {
                    insertCore(&((*node)->left), depth+1, point, id);
                }
                else 
                {
                    insertCore(&((*node)->right), depth+1, point, id);    
                }
            }
            else
            {
                if(point.z < ((*node)->point.z)) 
                {
                    insertCore(&((*node)->left), depth+1, point, id);
                }
                else 
                {
                    insertCore(&((*node)->right), depth+1, point, id);    
                }
            }    
        }    
    }

    void insert(pcl::PointXYZI point, int id)
    {
        insertCore(&root, 0, point, id);
    }

    void searchCore(pcl::PointXYZI target, Node* node, int depth, float distanceTol, std::vector<int> &ids)
    {
        if(node!=NULL)
        {
            if( (node->point.x >= (target.x - distanceTol)) && (node->point.x <= (target.x + distanceTol)) && (node->point.y >= (target.y - distanceTol)) && (node->point.y <= (target.y + distanceTol)))
            {
                float dist = sqrt(pow((node->point.x - target.x), 2) + pow((node->point.y - target.y), 2));
                if(dist<=distanceTol) 
                {
                    ids.push_back(node->id);
                }
            }

            if(depth % 2 == 0)
            {
                if((target.x - distanceTol) < node->point.x) 
                {
                    searchCore(target, node->left, depth+1, distanceTol, ids);
                }
                if((target.x + distanceTol) > node->point.x) 
                {
                    searchCore(target, node->right, depth+1, distanceTol, ids);
                }
            }
            else
            {
                if((target.y - distanceTol) < node->point.y) 
                {
                    searchCore(target, node->left, depth+1, distanceTol, ids);
                }
                if((target.y + distanceTol) > node->point.y) 
                {
                    searchCore(target, node->right, depth+1, distanceTol, ids);
                }
            }
        }
    }

    std::vector<int> search(pcl::PointXYZI target, float distanceTol)
    {
        std::vector<int> ids;
        searchCore(target, root, 0, distanceTol, ids);
        return ids;
    }
};