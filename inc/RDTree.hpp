#pragma once

#include <iostream>
#include <memory>
#include <algorithm>
#include <limits>
#include <cmath>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

/// @brief a node for an RDTree data structure.
/// @tparam T the type of data.
/// @tparam N the number of data elements.
template <typename T, size_t N>
struct RDTreeNode
{
  typedef std::array<T, N> RDTreeNodeData;

public:
  std::weak_ptr<RDTreeNode> mParent;
  std::shared_ptr<RDTreeNode> mLeftChild, mRightChild;
  RDTreeNodeData mData;

public:
  /// @brief constructor for the rd tree node.
  /// @param parent the parent of the node.
  /// @param leftChild the left child of the node.
  /// @param rightChild the right child of the node.
  /// @param data the data of the node.
  RDTreeNode(
      std::weak_ptr<RDTreeNode> parent,
      std::shared_ptr<RDTreeNode> leftChild,
      std::shared_ptr<RDTreeNode> rightChild,
      const RDTreeNodeData &data) : mParent(parent),
                                    mLeftChild(leftChild),
                                    mRightChild(rightChild),
                                    mData(data) {}

public:
  /// @brief create a root node.
  /// @param data the data for the root node.
  /// @return the constructed root node as smart pointer.
  static std::shared_ptr<RDTreeNode> createRoot(const RDTreeNodeData &data)
  {
    return std::make_shared<RDTreeNode>(
        std::shared_ptr<RDTreeNode>(nullptr),
        std::shared_ptr<RDTreeNode>(nullptr),
        std::shared_ptr<RDTreeNode>(nullptr),
        data);
  }

  /// @brief creates a child node.
  /// @param data the data for the child node.
  /// @param parent the parent of the child node.
  /// @return the constructed child node as smart pointer.
  static std::shared_ptr<RDTreeNode> createChild(const RDTreeNodeData &data,
                                                 std::weak_ptr<RDTreeNode> parent)
  {
    return std::make_shared<RDTreeNode>(
        parent,
        std::shared_ptr<RDTreeNode>(nullptr),
        std::shared_ptr<RDTreeNode>(nullptr),
        data);
  }

public:
  /// @brief checks if the node has a parent.
  /// @return true if the node has a parent.
  inline bool hasParent(void) const noexcept
  {
    return !(mParent.expired() || mParent.lock().get() == nullptr);
  }

  /// @brief checks if the node has a left child.
  /// @return true if there is a left child.
  inline bool hasLeftChild(void) const noexcept
  {
    return mLeftChild.get() != nullptr;
  }

  /// @brief checks if the node has a right child.
  /// @return true if the node has a right child.
  inline bool hasRightChild(void) const noexcept
  {
    return mRightChild.get() != nullptr;
  }

  /// @brief gets the left child.
  /// @return the left child.
  inline std::shared_ptr<RDTreeNode<T, N>> getLeftChild(void)
  {
    return mLeftChild;
  }

  /// @brief gets the right child.
  /// @return the right child.
  inline std::shared_ptr<RDTreeNode<T, N>> getRightChild(void)
  {
    return mRightChild;
  }

  /// @brief sets the left child.
  /// @param leftChild the left child to set.
  /// @return the current instance.
  inline RDTreeNode<T, N> &setLeftChild(const std::shared_ptr<RDTreeNode<T, N>> &leftChild)
  {
    mLeftChild = leftChild;

    return *this;
  }

  /// @brief sets the right child.
  /// @param rightChild the right child to set.
  /// @return the current instance.
  inline RDTreeNode<T, N> &setRightChild(const std::shared_ptr<RDTreeNode<T, N>> &rightChild)
  {
    mRightChild = rightChild;

    return *this;
  }

  /// @brief gets the parent.
  /// @return the parent.
  inline std::weak_ptr<RDTreeNode<T, N>> getParent(void)
  {
    return mParent;
  }

  /// @brief gets the data.
  /// @return the data.
  inline const std::array<T, N> &getData(void) const noexcept
  {
    return mData;
  }

  /// @brief gets the data.
  /// @return the data.
  inline std::array<T, N> &getData(void) noexcept
  {
    return mData;
  }

  /// @brief gets the nth element of the data.
  /// @param n the index to get.
  /// @return the element at the given index.
  inline const T &get(const size_t n) const
  {
    return mData.at(n);
  }

  /// @brief calculates the distance to the other given point.
  /// @param other the other point.
  /// @return the distance to the other point.
  inline T distanceTo(const std::array<T, N> &other) const
  {
    T sum = static_cast<T>(0);

    for (size_t i = 0; i < N; ++i)
      sum += std::pow(get(i) - other.at(i), static_cast<T>(2));

    return std::sqrt(sum);
  }
};

/// @brief the RDTree data structure.
/// @tparam T the type of data.
/// @tparam N the number of data elements.
template <typename T, size_t N>
class RDTree
{

protected:
  std::shared_ptr<RDTreeNode<T, N>> mRoot;
  size_t mSize;

public:
  /// @brief Constructs a RDTree.
  /// @param root the root element for the tree.
  /// @param size the size of the tree.
  RDTree(
      const std::shared_ptr<RDTreeNode<T, N>> &root,
      const size_t size) : mRoot(root), mSize(size)
  {
  }

public:
  /// @brief Constructs an empty RDTree.
  /// @return the constructed empty RDTree.
  static RDTree<T, N> empty(void) noexcept
  {
    return RDTree(std::shared_ptr<RDTreeNode<T, N>>(nullptr), 0);
  }

public:
  /// @brief gets the size of the tree.
  /// @return the size of the tree.
  inline size_t getSize(void) const noexcept
  {
    return mSize;
  }

  /// @brief if the current tree is empty.
  /// @return true if the tree is empty.
  inline bool isEmpty(void) const noexcept
  {
    return mRoot.get() == nullptr;
  }

protected:
  /// @brief inserts a root into the RDTree.
  /// @param data the data for the rood child/
  /// @return the current RDTree instance.
  RDTree<T, N> &insertRoot(const std::array<T, N> &data)
  {
    mRoot = RDTreeNode<T, N>::createRoot(data);
    return *this;
  }

  /// @brief inserts a child into the RDTree.
  /// @param data the data to insert into the RDTree.
  /// @return the current RDTree instance.
  RDTree<T, N> &insertChild(const std::array<T, N> &data)
  {
    std::shared_ptr<RDTreeNode<T, N>> currentNode = mRoot;

    for (size_t n = 0; currentNode.get() != nullptr; n = (n + 1) % N)
    {
      // Gets the values to compare.
      const T &a = currentNode->get(n);
      const T &b = data.at(n);

      // If the value of the nth value in the current node is greater than
      //  the nth value in the data, we insert it in the left child.
      if (a > b)
      {
        if (currentNode->getLeftChild().get() != nullptr)
        {
          currentNode = currentNode->getLeftChild();
        }
        else
        {
          currentNode->setLeftChild(RDTreeNode<T, N>::createChild(data, currentNode));
          break;
        }
      }
      else
      {
        if (currentNode->getRightChild().get() != nullptr)
        {
          currentNode = currentNode->getRightChild();
        }
        else
        {
          currentNode->setRightChild(RDTreeNode<T, N>::createChild(data, currentNode));
          break;
        }
      }
    }

    return *this;
  }

  /// @brief prints the given node with the given indentation (through n).
  /// @param node the node to print.
  /// @param n the indentation number.
  void print(const std::shared_ptr<RDTreeNode<T, N>> &node, const size_t n) const
  {
    if (node.get() == nullptr)
    {
      return;
    }

    for (size_t i = 0; i < n; ++i)
    {
      std::cout << "  ";
    }

    std::cout << '{';

    for (size_t i = 0; i < N; ++i)
    {
      std::cout << node->get(i);
      if (i + 1 != N)
        std::cout << ", ";
    }

    std::cout << '}' << std::endl;

    print(node->getLeftChild(), n + 1);
    print(node->getRightChild(), n + 1);
  }

public:
  /// @brief inserts a datapoint into the RDTree.
  /// @param data the data to insert.
  /// @return the current RDTree instance.
  RDTree<T, N> &insert(const std::array<T, N> &data)
  {
    ++this->mSize;
    return isEmpty() ? insertRoot(data) : insertChild(data);
  }

  /// @brief checks if the tree contains the given data points.
  /// @param data the data to check for.
  /// @return true if the tree contains the given data.
  bool contains(const std::array<T, N> &data)
  {
    std::shared_ptr<RDTreeNode<T, N>> currentNode = mRoot;

    for (size_t n = 0; currentNode.get() != nullptr; n = (n + 1) % N)
    {
      if (std::equal(data.begin(), data.end(), currentNode->getData().begin()))
      {
        return true;
      }

      // Gets the values to compare.
      const T &a = currentNode->get(n);
      const T &b = data.at(n);

      // If the value of the nth value in the current node is greater than
      //  the nth value in the data, we insert it in the left child.
      if (a > b)
      {
        if (currentNode->getLeftChild().get() != nullptr)
        {
          currentNode = currentNode->getLeftChild();
        }
        else
        {
          break;
        }
      }
      else
      {
        if (currentNode->getRightChild().get() != nullptr)
        {
          currentNode = currentNode->getRightChild();
        }
        else
        {
          break;
        }
      }
    }

    return false;
  }

  /// @brief prints the entire tree.
  /// @return the current tree instance.
  RDTree<T, N> &print(void)
  {
    print(mRoot, 0);

    return *this;
  }

  /// @brief prints the entire tree.
  /// @return the current tree instance.
  const RDTree<T, N> &print(void) const
  {
    print(mRoot, 0);

    return *this;
  }

  const std::array<T, N> &nearest(const std::array<T, N> &data) const
  {
    std::shared_ptr<RDTreeNode<T, N>> minDistanceNode = std::shared_ptr<RDTreeNode<T, N>>(nullptr);
    T minDistance = std::numeric_limits<T>::max();

    std::shared_ptr<RDTreeNode<T, N>> currentNode = mRoot;
    for (size_t n = 0; currentNode.get() != nullptr; n = (n + 1) % N)
    {
      // Checks if the distance of the current node is more close.
      const T distance = currentNode->distanceTo(data);
      if (distance < minDistance)
      {
        minDistance = distance;
        minDistanceNode = currentNode;
      }

      // Gets the values to compare.
      const T &a = currentNode->get(n);
      const T &b = data.at(n);

      // If the value of the nth value in the current node is greater than
      //  the nth value in the data, we insert it in the left child.
      if (a > b)
      {
        if (currentNode->getLeftChild().get() != nullptr)
        {
          currentNode = currentNode->getLeftChild();
        }
        else
        {
          break;
        }
      }
      else
      {
        if (currentNode->getRightChild().get() != nullptr)
        {
          currentNode = currentNode->getRightChild();
        }
        else
        {
          break;
        }
      }
    }

    return minDistanceNode->getData();
  }

public:
  /// @brief an iterator class for traversal of the tree.
  class iterator
  {
    using iterator_category = std::forward_iterator_tag;
    using difference_type = size_t;
    using value_type = std::array<T, N>;
    using pointer = std::array<T, N> *;
    using reference = std::array<T, N> &;

  private:
    std::shared_ptr<RDTreeNode<T, N>> mNode;
    RDTree<T, N> *mTree;

  public:
    explicit iterator(
        const std::shared_ptr<RDTreeNode<T, N>> &node,
        RDTree<T, N> *tree) : mNode(node),
                              mTree(tree)
    {
    }

    /// @brief traverses the tree.
    /// @return the reference to the current iterator.
    iterator &operator++(void)
    {
      /*
      Luke Binary-Tree Traversal
      Perform the following actions for each iteration:
        1. If there is a left child, set it as current and repeat.
        2. If there is a right child, set it as current and repeat.
        3. Find parent node which has a right child that isn't the current node and set it's right child as current and repeat.
        4. If no matching parent was found, exit, we've reached the last leaf.
      */

      // If there is a left child, traverse into it.
      if (mNode->hasLeftChild())
      {
        mNode = mNode->getLeftChild();
        return *this;
      }

      // If there is a right child, traverse into it.
      if (mNode->hasRightChild())
      {
        mNode = mNode->getRightChild();
        return *this;
      }

      // Find a parent node that has a right child we can traverse.
      mNode = [](std::shared_ptr<RDTreeNode<T, N>> current)
      {
        while (true)
        {
          if (not current->hasParent())
          {
            return std::shared_ptr<RDTreeNode<T, N>>(nullptr);
          }

          std::shared_ptr<RDTreeNode<T, N>> parent = current->getParent().lock();

          if (parent->hasRightChild() and parent->getRightChild() != current)
          {
            return parent->getRightChild();
          }

          current = parent;
        };
      }(mNode);
      return *this;
    }

    /// @brief traverses the tree but also returning the unmodified iterator.
    /// @param ignored.
    /// @return the unmodified iterator.
    iterator operator++(int)
    {
      iterator result = *this;

      ++(*this);

      return result;
    }

    /// @brief Overrides the equals operator.
    /// @param other the other iterator to compare to.
    /// @return if the two iterators are equal.
    bool operator==(const iterator &other) const noexcept
    {
      return (mTree == other.mTree) and (mNode == other.mNode);
    }

    /// @brief Overrides the unequal operator.
    /// @param other the other iterator to compare to.
    /// @return if the two iterators are unequal.
    bool operator!=(const iterator &other) const noexcept
    {
      return !((*this) == other);
    }

    /// @brief gets the reference to the current value in the iterator.
    /// @return the reference.
    reference &operator*(void)
    {
      return mNode->getData();
    }

    /// @brief gets the pointer to the current value in the iterator.
    /// @return the pointer.
    pointer operator->(void)
    {
      return mNode.get();
    }
  };

  /// @brief gets the beginning of the tree traversal iterator.
  /// @return the beginning of the iterator.
  iterator begin(void)
  {
    return iterator(mRoot, this);
  }

  /// @brief gets the end of the tree traversal iterator.
  /// @return the end of the iterator.
  iterator end(void)
  {
    return iterator(std::shared_ptr<RDTreeNode<T, N>>(nullptr), this);
  }
};

typedef RDTreeNode<double, 2> RDTreeNode2D;
typedef RDTreeNode<double, 3> RDTreeNode3D;

/// @brief a converter for converting lidar measurements to a rd tree.
class RDTreeFromLidarConverter
{
protected:
  bool mDisposeZeroDistances;

public:
  /// @brief constructs a rd tree from lidar converter with the given options.
  /// @param disposeZeroDistances whether or not to dispose zero distance entries.
  RDTreeFromLidarConverter(bool disposeZeroDistances) : mDisposeZeroDistances(disposeZeroDistances)
  {
  }

public:
  /// @brief constructs a rd tree from lidar converter with default options.
  /// @return the constructed converter.
  static RDTreeFromLidarConverter defaults(void) noexcept
  {
    return RDTreeFromLidarConverter(true);
  }

public:
  /// @brief converts lidar measurements to a rd tree.
  /// @tparam T the type of numbers to use.
  /// @param nodes the nodes.
  /// @param nNodes the number of nodes.
  /// @return the rd tree containing the points measured by the lidar.
  template <typename T>
  RDTree<T, 2> convertHQ(const sl_lidar_response_measurement_node_hq_t *nodes, const size_t nNodes)
  {
    RDTree<T, 2> tree = RDTree<T, 2>::empty();

    for (size_t i = 0; i < nNodes; ++i)
    {
      const sl_lidar_response_measurement_node_hq_t *node = &nodes[i];

      // Disposes zero distance points, if configured to do so.
      if (mDisposeZeroDistances and node->dist_mm_q2 == 0)
      {
        continue;
      }

      // Calculates the angle and the distance.
      const T angle = (static_cast<T>(node->angle_z_q14) * static_cast<T>(90)) / static_cast<T>(16384);
      const T distance = (static_cast<T>(node->dist_mm_q2) / static_cast<T>(4));

      // Calculates the X and Y coordinates from the polar angle and distance.
      const T x = distance * std::cos(angle);
      const T y = distance * std::sin(angle);

      // Inserts the point into the tree.
      tree.insert({x, y});
    }

    return tree;
  }
};