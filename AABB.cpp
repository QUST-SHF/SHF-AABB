#include "AABB.h"

namespace aabb
{
	AABB::AABB()
	{

	}

	AABB::AABB(unsigned int dimension)
	{
		assert(dimension >= 2);
		lowerBound.resize(dimension);
		upperBound.resize(dimension);
	}

	AABB::AABB(const std::vector<double>& lowerBound_, const std::vector<double>& upperBound_) :
		lowerBound(lowerBound_), upperBound(upperBound_)
	{
		//验证边界向量的维数
		if (lowerBound.size() != upperBound.size())
		{
			throw std::invalid_argument("[ERROR]: Dimension mismatch!");
		}
		//验证上界超过下界
		for (unsigned int i = 0; i < lowerBound.size(); i++)
		{
			//验证边界
			if (lowerBound[i] > upperBound[i])
			{
				throw std::invalid_argument("[ERROR]: AABB lower bound is greater than the upper bound!");
			}
		}
		surfaceArea = computeSurfaceArea();
		centre = computeCentre();
	}

	double AABB::computeSurfaceArea() const
	{
		//所有边的体积之和
		double sum = 0;
		//保持一个维度不变，然后乘以所有其他的
		for (unsigned int d1 = 0; d1 < lowerBound.size(); d1++)
		{
			//当前边的体积
			double product = 1;
			for (unsigned int d2 = 0; d2 < lowerBound.size(); d2++)
			{
				if (d1 == d2)
					continue;
				double dx = upperBound[d2] - lowerBound[d2];
				product *= dx;
			}
			sum += product;
		}
		return 2.0 * sum;
	}

	double AABB::getSurfaceArea() const
	{
		return surfaceArea;
	}

	void AABB::merge(const AABB& aabb1, const AABB& aabb2)
	{
		assert(aabb1.lowerBound.size() == aabb2.lowerBound.size());
		assert(aabb1.upperBound.size() == aabb2.upperBound.size());
		lowerBound.resize(aabb1.lowerBound.size());
		upperBound.resize(aabb1.upperBound.size());
		for (unsigned int i = 0; i < lowerBound.size(); i++)
		{
			lowerBound[i] = std::min(aabb1.lowerBound[i], aabb2.lowerBound[i]);
			upperBound[i] = std::max(aabb1.upperBound[i], aabb2.upperBound[i]);
		}
		surfaceArea = computeSurfaceArea();
		centre = computeCentre();
	}

	bool AABB::contains(const AABB& aabb) const
	{
		assert(aabb.lowerBound.size() == lowerBound.size());
		for (unsigned int i = 0; i < lowerBound.size(); i++)
		{
			if (aabb.lowerBound[i] < lowerBound[i]) return false;
			if (aabb.upperBound[i] > upperBound[i]) return false;
		}
		return true;
	}

	bool AABB::overlaps(const AABB& aabb, bool touchIsOverlap) const
	{
		assert(aabb.lowerBound.size() == lowerBound.size());
		bool rv = true;
		if (touchIsOverlap)
		{
			for (unsigned int i = 0; i < lowerBound.size(); ++i)
			{
				if (aabb.upperBound[i]<lowerBound[i] || aabb.lowerBound[i]>upperBound[i])
				{
					rv = false;
					break;
				}
			}
		}
		else
		{
			for (unsigned int i = 0; i < lowerBound.size(); ++i)
			{
				if (aabb.upperBound[i] <= lowerBound[i] || aabb.lowerBound[i] >= upperBound[i])
				{
					rv = false;
					break;
				}
			}
		}
		return rv;
	}

	std::vector<double> AABB::computeCentre()
	{
		std::vector<double> position(lowerBound.size());
		for (unsigned int i = 0; i < position.size(); i++)
			position[i] = 0.5*(lowerBound[i] + upperBound[i]);
		return position;
	}

	void AABB::setDimension(unsigned int dimension)
	{
		assert(dimension >= 2);
		lowerBound.resize(dimension);
		upperBound.resize(dimension);
	}

	Node::Node()
	{

	}

	bool Node::isLeaf() const
	{
		return(left == NULL_NODE);
	}

	Tree::Tree(unsigned int dimension_,
		double skinThickness_,
		unsigned int nParticles,
		bool touchIsOverlap_) :
		dimension(dimension_), isPeriodic(false), skinThickness(skinThickness_),
		touchIsOverlap(touchIsOverlap_)
	{
		//验证维度
		if (dimension < 2)
		{
			throw std::invalid_argument("[ERROR]: Invalid dimensionality!");
		}
		//初始化周期性向量
		periodicity.resize(dimension);
		std::fill(periodicity.begin(), periodicity.end(), false);
		//初始化树
		root = NULL_NODE;
		nodeCount = 0;
		nodeCapacity = nParticles;
		nodes.resize(nodeCapacity);
		//为空节点的列表建立一个链表
		for (unsigned int i = 0; i < nodeCapacity - 1; i++)
		{
			nodes[i].next = i + 1;
			nodes[i].height = -1;
		}
		nodes[nodeCapacity - 1].next = NULL_NODE;
		nodes[nodeCapacity - 1].height = -1;
		//分配第一个空节点的索引
		freeList = 0;
	}

	Tree::Tree(unsigned int dimension_,
		double skinThickness_,
		const std::vector<bool>& periodicity_,
		const std::vector<double>& boxSize_,
		unsigned int nParticles,
		bool touchIsOverlap_) :
		dimension(dimension_), skinThickness(skinThickness_),
		periodicity(periodicity_), boxSize(boxSize_),
		touchIsOverlap(touchIsOverlap_)
	{
		//验证维度
		if (dimension < 2)
		{
			throw std::invalid_argument("[ERROR]: Invalid dimensionality!");
		}
		//验证向量的维度
		if ((periodicity.size() != dimension) || (boxSize.size() != dimension))
		{
			throw std::invalid_argument("[ERROR]: Dimensionality mismatch!");
		}
		//初始化树
		root = NULL_NODE;
		touchIsOverlap = true;
		nodeCount = 0;
		nodeCapacity = nParticles;
		nodes.resize(nodeCapacity);
		//为空节点的列表建立一个链表
		for (unsigned int i = 0; i < nodeCapacity - 1; i++)
		{
			nodes[i].next = i + 1;
			nodes[i].height = -1;
		}
		nodes[nodeCapacity - 1].next = NULL_NODE;
		nodes[nodeCapacity - 1].height = -1;
		//分配第一个空节点的索引
		freeList = 0;
		//检查周期
		isPeriodic = false;
		posMinImage.resize(dimension);
		negMinImage.resize(dimension);
		for (unsigned int i = 0; i < dimension; i++)
		{
			posMinImage[i] = 0.5*boxSize[i];
			negMinImage[i] = -0.5*boxSize[i];
			if (periodicity[i])
				isPeriodic = true;
		}
	}

	void Tree::setPeriodicity(const std::vector<bool>& periodicity_)
	{
		periodicity = periodicity_;
	}

	void Tree::setBoxSize(const std::vector<double>& boxSize_)
	{
		boxSize = boxSize_;
	}

	unsigned int Tree::allocateNode()
	{
		//如果有需要就扩充节点池
		if (freeList == NULL_NODE)
		{
			assert(nodeCount == nodeCapacity);
			//免费列表是空的，重建一个更大的池
			nodeCapacity *= 2;
			nodes.resize(nodeCapacity);
			//为空节点的列表建立一个链表
			for (unsigned int i = nodeCount; i < nodeCapacity - 1; i++)
			{
				nodes[i].next = i + 1;
				nodes[i].height = -1;
			}
			nodes[nodeCapacity - 1].next = NULL_NODE;
			nodes[nodeCapacity - 1].height = -1;
			//分配第一个空节点的索引
			freeList = nodeCount;
		}
		//将一个节点从空闲列表中剥离出来
		unsigned int node = freeList;
		freeList = nodes[node].next;
		nodes[node].parent = NULL_NODE;
		nodes[node].left = NULL_NODE;
		nodes[node].right = NULL_NODE;
		nodes[node].height = 0;
		nodes[node].aabb.setDimension(dimension);
		nodeCount++;

		return node;
	}

	void Tree::freeNode(unsigned int node)
	{
		assert(node < nodeCapacity);
		assert(0 < nodeCount);
		nodes[node].next = freeList;
		nodes[node].height = -1;
		freeList = node;
		nodeCount--;
	}

	void Tree::insertParticle(unsigned int particle, std::vector<double>& position, double radius)
	{
		//确保粒子还不存在
		if (particleMap.count(particle) != 0)
		{
			throw std::invalid_argument("[ERROR]: Particle already exists in tree!");
		}
		//验证位置矢量的维数
		if (position.size() != dimension)
		{
			throw std::invalid_argument("[ERROR]: Dimensionality mismatch!");
		}
		//为粒子分配一个新节点
		unsigned int node = allocateNode();
		//AABB在每个维度上的大小
		std::vector<double> size(dimension);
		//计算AABB的极限
		for (unsigned int i = 0; i < dimension; i++)
		{
			nodes[node].aabb.lowerBound[i] = position[i] - radius;
			nodes[node].aabb.upperBound[i] = position[i] + radius;
			size[i] = nodes[node].aabb.upperBound[i] - nodes[node].aabb.lowerBound[i];
		}
		//使AABB更充实
		for (unsigned int i = 0; i < dimension; i++)
		{
			nodes[node].aabb.lowerBound[i] -= skinThickness*size[i];
			nodes[node].aabb.upperBound[i] += skinThickness*size[i];
		}
		nodes[node].aabb.surfaceArea = nodes[node].aabb.computeSurfaceArea();
		nodes[node].aabb.centre = nodes[node].aabb.computeCentre();
		//高度设置为0
		nodes[node].height = 0;
		//在树中插入一个新的叶子节点
		insertLeaf(node);
		//将新粒子添加到映射中
		particleMap.insert(std::map<unsigned int, unsigned int>::value_type(particle, node));
		//存储粒子的索引
		nodes[node].particle = particle;
	}

	void Tree::insertParticle(unsigned int particle, std::vector<double>& lowerBound, std::vector<double>& upperBound)
	{
		//确保粒子还不存在
		if (particleMap.count(particle) != 0)
		{
			throw std::invalid_argument("[ERROR]: Particle already exists in tree!");
		}
		//验证边界向量的维度
		if ((lowerBound.size() != dimension) || (upperBound.size() != dimension))
		{
			throw std::invalid_argument("[ERROR]: Dimensionality mismatch!");
		}
		//为粒子分配新的节点
		unsigned int node = allocateNode();
		//AABB在每个维度上的大小
		std::vector<double> size(dimension);
		//计算AABB的极限
		for (unsigned int i = 0; i < dimension; i++)
		{
			//验证边界
			if (lowerBound[i] > upperBound[i])
			{
				throw std::invalid_argument("[ERROR]: AABB lower bound is greater than the upper bound!");
			}
			nodes[node].aabb.lowerBound[i] = lowerBound[i];
			nodes[node].aabb.upperBound[i] = upperBound[i];
			size[i] = upperBound[i] - lowerBound[i];
		}
		//使AABB更充实
		for (unsigned int i = 0; i < dimension; i++)
		{
			nodes[node].aabb.lowerBound[i] -= skinThickness*size[i];
			nodes[node].aabb.upperBound[i] += skinThickness*size[i];
		}
		nodes[node].aabb.surfaceArea = nodes[node].aabb.computeSurfaceArea();
		nodes[node].aabb.centre = nodes[node].aabb.computeCentre();
		//高度设置为0
		nodes[node].height = 0;
		//在树中插入一个新的叶子节点
		insertLeaf(node);
		//将新粒子添加到映射中
		particleMap.insert(std::map<unsigned int, unsigned int>::value_type(particle, node));
		//存储粒子索引
		nodes[node].particle = particle;
	}

	unsigned int Tree::nParticles()
	{
		return particleMap.size();
	}

	void Tree::removeParticle(unsigned int particle)
	{
		//映射迭代器
		std::map<unsigned int, unsigned int>::iterator it;
		//找粒子
		it = particleMap.find(particle);
		//粒子不存在
		if (it == particleMap.end())
		{
			throw std::invalid_argument("[ERROR]: Invalid particle index!");
		}
		//提取节点索引
		unsigned int node = it->second;
		//从映射中删除这个粒子
		particleMap.erase(it);
		assert(node < nodeCapacity);
		assert(nodes[node].isLeaf());
		removeLeaf(node);
		freeNode(node);
	}

	void Tree::removeAll()
	{
		//迭代器指向粒子映射的开始
		std::map<unsigned int, unsigned int>::iterator it = particleMap.begin();
		//遍历映射
		while (it!=particleMap.end())
		{
			//提取节点索引
			unsigned int node = it->second;
			assert(node < nodeCapacity);
			assert(nodes[node].isLeaf());
			removeLeaf(node);
			freeNode(node);
			it++;
		}
		//清除粒子映射
		particleMap.clear();
	}

	bool Tree::updateParticle(unsigned int particle, std::vector<double>& position, double radius,
		bool alwaysReinsert)
	{
		//验证位置矢量的维数
		if (position.size() != dimension)
		{
			throw std::invalid_argument("[ERROR]: Dimensionality mismatch!");
		}
		//AABB边界向量
		std::vector<double> lowerBound(dimension);
		std::vector<double> upperBound(dimension);
		//计算AABB的极限
		for (unsigned int i = 0; i < dimension; i++)
		{
			lowerBound[i] = position[i] - radius;
			upperBound[i] = position[i] + radius;
		}
		//更新粒子
		return updateParticle(particle, lowerBound, upperBound, alwaysReinsert);
	}

	bool Tree::updateParticle(unsigned int particle, std::vector<double>& lowerBound,
		std::vector<double>& upperBound, bool alwaysReinsert)
	{
		//验证边界向量的维度
		if ((lowerBound.size() != dimension) && (upperBound.size() != dimension))
		{
			throw std::invalid_argument("[ERROR]: Dimensionality mismatch!");
		}
		//映射迭代器
		std::map<unsigned int, unsigned int>::iterator it;
		//找粒子
		it = particleMap.find(particle);
		//粒子不存在
		if (it == particleMap.end())
		{
			throw std::invalid_argument("[ERROR]: Invalid particle index!");
		}
		//提取节点索引
		unsigned int node = it->second;
		assert(node < nodeCapacity);
		assert(nodes[node].isLeaf());
		//AABB在每个维度上的大小
		std::vector<double> size(dimension);
		//计算AABB的极限
		for (unsigned int i = 0; i < dimension; i++)
		{
			//验证边界
			if (lowerBound[i]>upperBound[i])
			{
				throw std::invalid_argument("[ERROR]: AABB lower bound is greater than the upper bound!");
			}
			size[i] = upperBound[i] - lowerBound[i];
		}
		//创建新的AABB
		AABB aabb(lowerBound, upperBound);
		//如果粒子仍在它的肥育中，就不需要更新
		if (!alwaysReinsert && nodes[node].aabb.contains(aabb)) return false;
		//移除当前的叶子
		removeLeaf(node);
		//使新的AABB更加充实
		for (unsigned int i = 0; i < dimension; i++)
		{
			aabb.lowerBound[i] -= skinThickness*size[i];
			aabb.upperBound[i] += skinThickness*size[i];
		}
		//分配新的AABB
		nodes[node].aabb = aabb;
		//更新表面区域和中心体
		nodes[node].aabb.surfaceArea = nodes[node].aabb.computeSurfaceArea();
		nodes[node].aabb.centre = nodes[node].aabb.computeCentre();
		//插入一个新的叶子节点
		insertLeaf(node);

		return true;
	}

	std::vector<unsigned int> Tree::query(unsigned int particle)
	{
		//确认这是一个验证的粒子
		if (particleMap.count(particle) == 0)
		{
			throw std::invalid_argument("[ERROR]: Invalid particle index!");
		}
		//粒子AABB与其他粒子的测试重叠
		return query(particle, nodes[particleMap.find(particle)->second].aabb);
	}

	std::vector<unsigned int> Tree::query(unsigned int particle, const AABB& aabb)
	{
		std::vector<unsigned int> stack;
		stack.reserve(256);
		stack.push_back(root);
		std::vector<unsigned int> particles;
		while (stack.size()>0)
		{
			unsigned int node = stack.back();
			stack.pop_back();
			//复制AABB
			AABB nodeAABB = nodes[node].aabb;
			if (node == NULL_NODE) continue;
			if (isPeriodic)
			{
				std::vector<double> separation(dimension);
				std::vector<double> shift(dimension);
				for (unsigned int i = 0; i < dimension; i++)
					separation[i] = nodeAABB.centre[i] - aabb.centre[i];
				bool isShifted = minimumImage(separation, shift);
				//平移AABB
				if (isShifted)
				{
					for (unsigned int i = 0; i < dimension; i++)
					{
						nodeAABB.lowerBound[i] += shift[i];
						nodeAABB.upperBound[i] += shift[i];
					}
				}
			}
			//AABBs之间的重叠测试
			if (aabb.overlaps(nodeAABB, touchIsOverlap))
			{
				//检查我们是否在叶子节点上
				if (nodes[node].isLeaf())
				{
					//不能与自己互动
					if (nodes[node].particle != particle)
					{
						particles.push_back(nodes[node].particle);
					}
				}
				else
				{
					stack.push_back(nodes[node].left);
					stack.push_back(nodes[node].right);
				}
			}
		}

		return particles;
	}

	std::vector<unsigned int> Tree::query(const AABB& aabb)
	{
		//确认树不是空的
		if (particleMap.size() == 0)
		{
			return std::vector<unsigned int>();
		}
		//AABB与所有粒子的测试重叠
		return query(std::numeric_limits<unsigned int>::max(), aabb);
	}

	const AABB& Tree::getAABB(unsigned int particle)
	{
		return nodes[particleMap[particle]].aabb;
	}

	void Tree::insertLeaf(unsigned int leaf)
	{
		if (root == NULL_NODE)
		{
			root = leaf;
			nodes[root].parent = NULL_NODE;
			return;
		}
		//为节点找到最好的兄弟姐妹
		AABB leafAABB = nodes[leaf].aabb;
		unsigned int index = root;
		while (!nodes[index].isLeaf())
		{
			//提取节点的子节点
			unsigned int left = nodes[index].left;
			unsigned int right = nodes[index].right;
			double surfaceArea = nodes[index].aabb.getSurfaceArea();
			AABB combinedAABB;
			combinedAABB.merge(nodes[index].aabb, leafAABB);
			double combinedSurfaceArea = combinedAABB.getSurfaceArea();
			//为这个节点和新叶子创建一个新双亲的成本
			double cost = 2.0 * combinedSurfaceArea;
			//把叶子推到树下的最小成本
			double inheritanceCost = 2.0 * (combinedSurfaceArea - surfaceArea);
			//向左转的成本
			double costLeft;
			if (nodes[left].isLeaf())
			{
				AABB aabb;
				aabb.merge(leafAABB, nodes[left].aabb);
				costLeft = aabb.getSurfaceArea() + inheritanceCost;
			}
			else
			{
				AABB aabb;
				aabb.merge(leafAABB, nodes[left].aabb);
				double oldArea = nodes[left].aabb.getSurfaceArea();
				double newArea = aabb.getSurfaceArea();
				costLeft = (newArea - oldArea) + inheritanceCost;
			}
			//向右转的成本
			double costRight;
			if (nodes[right].isLeaf())
			{
				AABB aabb;
				aabb.merge(leafAABB, nodes[right].aabb);
				costRight = aabb.getSurfaceArea() + inheritanceCost;
			}
			else
			{
				AABB aabb;
				aabb.merge(leafAABB, nodes[right].aabb);
				double oldArea = nodes[right].aabb.getSurfaceArea();
				double newArea = aabb.getSurfaceArea();
				costRight = (newArea - oldArea) + inheritanceCost;
			}
			//根据最低成本下降
			if ((cost < costLeft) && (cost < costRight)) break;
			//下降
			if (costLeft < costRight) index = left;
			else                      index = right;
		}
		unsigned int sibling = index;
		//创建一个新的双亲
		unsigned int oldParent = nodes[sibling].parent;
		unsigned int newParent = allocateNode();
		nodes[newParent].parent = oldParent;
		nodes[newParent].aabb.merge(leafAABB, nodes[sibling].aabb);
		nodes[newParent].height = nodes[sibling].height + 1;
		//兄弟姐妹不是根
		if (oldParent != NULL_NODE)
		{
			if (nodes[oldParent].left == sibling) nodes[oldParent].left = newParent;
			else                                  nodes[oldParent].right = newParent;
			nodes[newParent].left = sibling;
			nodes[newParent].right = leaf;
			nodes[sibling].parent = newParent;
			nodes[leaf].parent = newParent;
		}
		//兄弟姐妹是根
		else
		{
			nodes[newParent].left = sibling;
			nodes[newParent].right = leaf;
			nodes[sibling].parent = newParent;
			nodes[leaf].parent = newParent;
			root = newParent;
		}
		//走回树的高处和AABBs
		index = nodes[leaf].parent;
		while (index!=NULL_NODE)
		{
			index = balance(index);
			unsigned int left = nodes[index].left;
			unsigned int right = nodes[index].right;
			assert(left != NULL_NODE);
			assert(right != NULL_NODE);
			nodes[index].height = 1 + std::max(nodes[left].height, nodes[right].height);
			nodes[index].aabb.merge(nodes[left].aabb, nodes[right].aabb);
			index = nodes[index].parent;
		}
	}
	
	void Tree::removeLeaf(unsigned int leaf)
	{
		if (leaf == root)
		{
			root = NULL_NODE;
			return;
		}
		unsigned int parent = nodes[leaf].parent;
		unsigned int grandParent = nodes[parent].parent;
		unsigned int sibling;
		if (nodes[parent].left == leaf) sibling = nodes[parent].right;
		else                            sibling = nodes[parent].left;
		//破坏父结点并将兄弟姐妹连接到祖父结点
		if (grandParent!=NULL_NODE)
		{
			if (nodes[grandParent].left == parent) nodes[grandParent].left = sibling;
			else                                   nodes[grandParent].right = sibling;
			nodes[sibling].parent = grandParent;
			freeNode(grandParent);
			//调整祖先界限
			unsigned int index = grandParent;
			while (index!=NULL_NODE)
			{
				index = balance(index);
				unsigned int left = nodes[index].left;
				unsigned int right = nodes[index].right;
				nodes[index].aabb.merge(nodes[left].aabb, nodes[right].aabb);
				nodes[index].height = 1 + std::max(nodes[left].height, nodes[right].height);
				index = nodes[index].parent;
			}
		}
		else
		{
			root = sibling;
			nodes[sibling].parent = NULL_NODE;
			freeNode(parent);
		}
	}

	unsigned int Tree::balance(unsigned int node)
	{
		assert(node != NULL_NODE);
		if (nodes[node].isLeaf() || (nodes[node].height < 2))
			return node;
		unsigned int left = nodes[node].left;
		unsigned int right = nodes[node].right;
		assert(left < nodeCapacity);
		assert(right < nodeCapacity);
		int currentBalance = nodes[right].height - nodes[left].height;
		//旋转右分支
		if (currentBalance>1)
		{
			unsigned int rightLeft = nodes[right].left;
			unsigned int rightRight = nodes[right].right;
			assert(rightLeft < nodeCapacity);
			assert(rightRight < nodeCapacity);
			//交换节点和它的右子结点
			nodes[right].left = node;
			nodes[right].parent = nodes[node].parent;
			nodes[node].parent = right;
			//节点的老父节点现在应该指向它的右子节点
			if (nodes[right].parent!=NULL_NODE)
			{
				if (nodes[nodes[right].parent].left == node) nodes[nodes[right].parent].left = right;
				else
				{
					assert(nodes[nodes[right].parent].right == node);
					nodes[nodes[right].parent].right = right;
				}
			}
			else root = right;
			//旋转
			if (nodes[rightLeft].height > nodes[rightRight].height)
			{
				nodes[right].right = rightLeft;
				nodes[node].right = rightRight;
				nodes[rightRight].parent = node;
				nodes[node].aabb.merge(nodes[left].aabb, nodes[rightRight].aabb);
				nodes[right].aabb.merge(nodes[node].aabb, nodes[rightLeft].aabb);
			}
			else
			{
				nodes[right].right = rightRight;
				nodes[node].right = rightLeft;
				nodes[rightLeft].parent = node;
				nodes[node].aabb.merge(nodes[left].aabb, nodes[rightLeft].aabb);
				nodes[right].aabb.merge(nodes[node].aabb, nodes[rightRight].aabb);
				nodes[node].height = 1 + std::max(nodes[left].height, nodes[rightLeft].height);
				nodes[right].height = 1 + std::max(nodes[node].height, nodes[rightRight].height);
			}
			return right;
		}
		// 旋转左分支
		if (currentBalance < -1)
		{
			unsigned int leftLeft = nodes[left].left;
			unsigned int leftRight = nodes[left].right;
			assert(leftLeft < nodeCapacity);
			assert(leftRight < nodeCapacity);
			// 交换节点和它的右子结点
			nodes[left].left = node;
			nodes[left].parent = nodes[node].parent;
			nodes[node].parent = left;
			// 节点的老父节点现在应该指向它的右子节点
			if (nodes[left].parent != NULL_NODE)
			{
				if (nodes[nodes[left].parent].left == node) nodes[nodes[left].parent].left = left;
				else
				{
					assert(nodes[nodes[left].parent].right == node);
					nodes[nodes[left].parent].right = left;
				}
			}
			else root = left;
			// 旋转
			if (nodes[leftLeft].height > nodes[leftRight].height)
			{
				nodes[left].right = leftLeft;
				nodes[node].left = leftRight;
				nodes[leftRight].parent = node;
				nodes[node].aabb.merge(nodes[right].aabb, nodes[leftRight].aabb);
				nodes[left].aabb.merge(nodes[node].aabb, nodes[leftLeft].aabb);
				nodes[node].height = 1 + std::max(nodes[right].height, nodes[leftRight].height);
				nodes[left].height = 1 + std::max(nodes[node].height, nodes[leftLeft].height);
			}
			else
			{
				nodes[left].right = leftRight;
				nodes[node].left = leftLeft;
				nodes[leftLeft].parent = node;
				nodes[node].aabb.merge(nodes[right].aabb, nodes[leftLeft].aabb);
				nodes[left].aabb.merge(nodes[node].aabb, nodes[leftRight].aabb);
				nodes[node].height = 1 + std::max(nodes[right].height, nodes[leftLeft].height);
				nodes[left].height = 1 + std::max(nodes[node].height, nodes[leftRight].height);
			}
			return left;
		}
		return node;
	}

	unsigned int Tree::computeHeight() const
	{
		return computeHeight(root);
	}

	unsigned int Tree::computeHeight(unsigned int node) const
	{
		assert(node < nodeCapacity);
		if (nodes[node].isLeaf()) return 0;
		unsigned int height1 = computeHeight(nodes[node].left);
		unsigned int height2 = computeHeight(nodes[node].right);
		return 1 + std::max(height1, height2);
	}

	unsigned int Tree::getHeight() const
	{
		if (root == NULL_NODE) return 0;
		return nodes[root].height;
	}

	unsigned int Tree::getNodeCount() const
	{
		return nodeCount;
	}

	unsigned int Tree::computeMaximumBalance() const
	{
		unsigned int maxBalance = 0;
		for (unsigned int i = 0; i < nodeCapacity; i++)
		{
			if (nodes[i].height <= 1)
				continue;
			assert(nodes[i].isLeaf() == false);
			unsigned int balance = std::abs(nodes[nodes[i].left].height - nodes[nodes[i].right].height);
			maxBalance = std::max(maxBalance, balance);
		}
		return maxBalance;
	}

	double Tree::computeSurfaceAreaRatio() const
	{
		if (root == NULL_NODE) return 0.0;
		double rootArea = nodes[root].aabb.computeSurfaceArea();
		double totalArea = 0.0;
		for (unsigned int  i = 0; i < nodeCapacity; i++)
		{
			if (nodes[i].height < 0) continue;
			totalArea += nodes[i].aabb.computeSurfaceArea();
		}
		return totalArea / rootArea;
	}

	void Tree::validate() const
	{
#ifndef NDEBUG
		validateStructure(root);
		validateMetrics(root);
		unsigned int freeCount = 0;
		unsigned int freeIndex = freeList;
		while (freeIndex!=NULL_NODE)
		{
			assert(freeIndex < nodeCapacity);
			freeIndex = nodes[freeIndex].next;
			freeCount++;
		}
		assert(getHeight() == computeHeight());
		assert((nodeCount + freeCount) == nodeCapacity);
#endif
	}

	void Tree::rebuild()
	{
		std::vector<unsigned int> nodeIndices(nodeCount);
		unsigned int count = 0;
		for (unsigned int i = 0; i < nodeCapacity; i++)
		{
			//空节点
			if (nodes[i].height < 0) continue;
			if (nodes[i].isLeaf())
			{
				nodes[i].parent = NULL_NODE;
				nodeIndices[count] = i;
				count++;
			}
			else freeNode(i);
		}
		while (count > 1)
		{
			double minCost = std::numeric_limits<double>::max();
			int  iMin = -1, jMin = -1;
			for (unsigned int i = 0; i < count; i++)
			{
				AABB aabbi = nodes[nodeIndices[i]].aabb;
				for (unsigned int j = i + 1; j < count; j++)
				{
					AABB aabbj = nodes[nodeIndices[j]].aabb;
					AABB aabb;
					aabb.merge(aabbi, aabbj);
					double cost = aabb.getSurfaceArea();
					if (cost < minCost)
					{
						iMin = i;
						jMin = j;
						minCost = cost;
					}
				}
			}
			unsigned int index1 = nodeIndices[iMin];
			unsigned int index2 = nodeIndices[jMin];
			unsigned int parent = allocateNode();
			nodes[parent].left = index1;
			nodes[parent].right = index2;
			nodes[parent].height = 1 + std::max(nodes[index1].height, nodes[index2].height);
			nodes[parent].aabb.merge(nodes[index1].aabb, nodes[index2].aabb);
			nodes[parent].parent = NULL_NODE;
			nodes[index1].parent = parent;
			nodes[index2].parent = parent;
			nodeIndices[jMin] = nodeIndices[count - 1];
			nodeIndices[iMin] = parent;
			count--;
		}
		root = nodeIndices[0];
		validate();
	}

	void Tree::validateStructure(unsigned int node) const
	{
		if (node == NULL_NODE) return;
		if (node == root) assert(nodes[node].parent == NULL_NODE);
		unsigned int left = nodes[node].left;
		unsigned int right = nodes[node].right;
		if (nodes[node].isLeaf())
		{
			assert(left = NULL_NODE);
			assert(right = NULL_NODE);
			assert(nodes[node].height == 0);
			return;
		}
		assert(left < nodeCapacity);
		assert(right < nodeCapacity);
		assert(nodes[left].parent == node);
		assert(nodes[right].parent == node);
		validateStructure(left);
		validateStructure(right);
	}

	void Tree::validateMetrics(unsigned int node) const
	{
		if (node == NULL_NODE) return;
		unsigned int left = nodes[node].left;
		unsigned int right = nodes[node].right;
		if (nodes[node].isLeaf())
		{
			assert(left == NULL_NODE);
			assert(right == NULL_NODE);
			assert(nodes[node].height == 0);
			return;
		}
		assert(left < nodeCapacity);
		assert(right < nodeCapacity);
		int height1 = nodes[left].height;
		int height2 = nodes[right].height;
		int height = 1 + std::max(height1, height2);
		(void)height;
		assert(nodes[node].height == height);
		AABB aabb;
		aabb.merge(nodes[left].aabb, nodes[right].aabb);
		for (unsigned int i = 0; i < dimension; i++)
		{
			assert(aabb.lowerBound[i] == nodes[node].aabb.lowerBound[i]);
			assert(aabb.upperBound[i] == nodes[node].aabb.upperBound[i]);
		}
		validateMetrics(left);
		validateMetrics(right);
	}

	void Tree::periodicBoundaries(std::vector<double>& position)
	{
		for (unsigned int i = 0; i < dimension; i++)
		{
			if (position[i] < 0)
			{
				position[i] += boxSize[i];
			}
			else
			{
				if (position[i] >= boxSize[i])
				{
					position[i] -= boxSize[i];
				}
			}
		}
	}

	bool Tree::minimumImage(std::vector<double>& separation, std::vector<double>& shift)
	{
		bool isShifted = false;
		for (unsigned int i = 0; i < dimension; i++)
		{
			if (separation[i] < negMinImage[i])
			{
				separation[i] += periodicity[i] * boxSize[i];
				shift[i] = periodicity[i] * boxSize[i];
				isShifted = true;
			}
			else
			{
				if (separation[i] >= posMinImage[i])
				{
					separation[i] -= periodicity[i] * boxSize[i];
					shift[i] = -periodicity[i] * boxSize[i];
					isShifted = true;
				}
			}
		}
		return isShifted;
	}
}
