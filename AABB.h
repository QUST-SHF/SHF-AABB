#pragma once
#ifndef _AABB_H
#define _AABB_H

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <vector>

const unsigned int NULL_NODE = 0xffffffff;

namespace aabb 
{
	/*轴对齐的边界框（AABBs）存储在二维或三维空间中一个物体的最小的或正交的盒子信息（边界框是一个矩形）。
	Class成员函数提供了合并AABB对象的功能，并测试与其他AABBs的重叠。*/
	class AABB
	{
	public:
		AABB();                                                            //无参数
		AABB(unsigned int);                                                //参数是维度
		AABB(const std::vector<double>&, const std::vector<double>&);      //每个维度的上下边界
		double computeSurfaceArea() const;                                 //计算盒子的表面积
		double getSurfaceArea() const;                                     //得到盒子的表面积
		void merge(const AABB&, const AABB&);                              //合并AABB盒子
		bool contains(const AABB&) const;                                  //测试AABB是否包含在其中，返回值是否包含
		bool overlaps(const AABB&, bool touchIsOverlap) const;             //测试AABB是否与它重叠，返回值为是否重叠
		std::vector<double> computeCentre();                               //计算AABB的中心，返回AABB中心的坐标向量
		void setDimension(unsigned int);                                   //设置AABB的维度
		std::vector<double> lowerBound;                                    //每个维度上AABB的下边界
		std::vector<double> upperBound;                                    //上边界
		std::vector<double> centre;                                        //AABB中心的位置
		double surfaceArea;                                                //AABB的表面积
	};
	/*树的每个节点都包含一个AABB对象，它对应于一个粒子，或者是一个粒子群，在模拟盒中。
	在储存之前，单个粒子的AABB对象被“养肥”，以避免在位移很小的情况下不断更新和重新平衡树。
	节点知道它们在树中的位置。isLeaf的成员函数允许树查询节点是否是叶子，也就是说，确定它是否包含单个粒子。*/
	struct Node
	{
		Node();                                                            //无参数
		AABB aabb;                                                         //养肥的AABB盒子
		unsigned int parent;                                               //父节点索引
		unsigned int next;                                                 //下一个节点索引
		unsigned int left;                                                 //左边孩子的索引
		unsigned int right;                                                //右边孩子的索引
		int height;                                                        //节点的高度，0是叶子节点，-1是自由节点
		unsigned int particle;                                             //节点包含粒子的索引（仅限于叶节点）
		bool isLeaf() const;                                               //检测节点是否是叶子节点
	};
	/*动态的AABB树是一个分层的数据结构，可以用来有效地查询模拟框中任意形状和大小的物体之间的重叠。
	为定期和非周期的盒子提供支持，以及带有部分周期性的盒子，例如在特定轴上的周期性。*/
	class Tree
	{
	public:
		Tree(unsigned int dimension_ = 3, double skinThickness_ = 0.05,
			unsigned int nParticles = 16, bool touchIsOverlap = true);     //非周期的树
		Tree(unsigned int, double, const std::vector<bool>&, const std::vector<double>&,
			unsigned int nParticles = 16, bool touchIsOverlap = true);     //自定义的周期，周期参数以及盒子大小参数
		void setPeriodicity(const std::vector<bool>&);                     //设置模拟框的周期性
		void setBoxSize(const std::vector<double>&);                       //设置模拟框的大小
		void insertParticle(unsigned int, std::vector<double>&, double);   //插入粒子到树中，参数是粒子索引，位置，半径
		void insertParticle(unsigned int, std::vector<double>&,            //将一个粒子插入到树中（带有边框的任意形状）
			std::vector<double>&);                                         //参数是粒子索引，每个维度的上下边界
		unsigned int nParticles();                                         //返回树中粒子的数量
		void removeParticle(unsigned int);                                 //从树中移除某个粒子
		void removeAll();                                                  //从树中移除所有
		bool updateParticle(unsigned int, std::vector<double>&, double,    //如果一个粒子在它的肥化的AABB外移动，更新树
			bool alwaysReinsert = false);                                  //参数是粒子索引，位置，半径。返回粒子是否重新插入
		bool updateParticle(unsigned int, std::vector<double>&,
			std::vector<double>&, bool alwaysReinsert = false);            //参数是粒子索引，每个维度上下边界，alwaysReinsert
		std::vector<unsigned int> query(unsigned int);                     //查询树以找到一个粒子的候选交互
		std::vector<unsigned int> query(unsigned int, const AABB&);        //查询树以找到AABB的候选交互
		std::vector<unsigned int> query(const AABB&);                      //查询树以找到AABB的候选交互
		const AABB& getAABB(unsigned int);                                 //得到一个粒子的AABB
		unsigned int getHeight() const;                                    //得到二叉树的高度
		unsigned int getNodeCount() const;                                 //得到树的节点个数
		unsigned int computeMaximumBalance() const;                        //计算树的最大平衡
		double computeSurfaceAreaRatio() const;                            //计算树的表面积比
		void validate() const;                                             //验证树
		void rebuild();                                                    //重建最优树
	private:
		unsigned int root;                                                 //根节点索引
		std::vector<Node> nodes;                                           //动态树
		unsigned int nodeCount;                                            //树中当前节点的数目
		unsigned int nodeCapacity;                                         //当前节点的容量
		unsigned int freeList;                                             //节点位于空闲列表顶部的位置
		unsigned int dimension;                                            //系统的维度
		bool isPeriodic;                                                   //系统是否在至少一个轴上是周期性的
		double skinThickness;                                              //养肥的AABBs的皮肤厚度，作为AABB基础长度的一小部分
		std::vector<bool> periodicity;                                     //系统是否在每个轴上都是周期性的
		std::vector<double> boxSize;                                       //每个维度的系统的大小
		std::vector<double> negMinImage;                                   //负的最小图像的位置
		std::vector<double> posMinImage;                                   //正的最小图像的位置
		std::map<unsigned int, unsigned int> particleMap;                  //粒子与节点索引之间的映射
		bool touchIsOverlap;                                               //触摸计数在树查询中是重叠的吗？
		unsigned int allocateNode();                                       //分配一个新的节点
		void freeNode(unsigned int);                                       //释放一个现有节点
		void insertLeaf(unsigned int);                                     //插入一个叶子到树中
		void removeLeaf(unsigned int);                                     //从树中移除一个叶子
		unsigned int balance(unsigned int);                                //平衡树
		unsigned int computeHeight() const;                                //计算树的高度
		unsigned int computeHeight(unsigned int) const;                    //计算子树的高度，参数是根节点索引
		void validateStructure(unsigned int) const;                        //断言子树有一个有效的结构
		void validateMetrics(unsigned int) const;                          //断言子树具有有效的度量标准
		void periodicBoundaries(std::vector<double>&);                     //应用周期边界条件，参数是位置向量
		bool minimumImage(std::vector<double>&, std::vector<double>&);     //计算最小图像分离，参数分离向量，平移向量
	};
}
#endif // !_AABB_H
