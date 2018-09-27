#include <fstream>
#include <iostream>

#include "AABB.h"
#include "MersenneTwister.h"

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif // !M_PI

//检查是否两个粒子重叠
bool overlaps(std::vector<double>&, std::vector<double>&, const std::vector<bool>&, const std::vector<double>&, double);
//计算最小图像分割向量
void minimumImage(std::vector<double>&, const std::vector<bool>&, const std::vector<double>&);
//应用周期边界条件
void periodicBoundaries(std::vector<double>&, const std::vector<bool>&, const std::vector<double>&);

int main(int argc, char** argv)
{
#ifdef COMMIT
	std::cout << "Git commit:" << COMMIT << "\n";
#endif
#ifdef BRANCH
	std::cout << "Git branch:" << BRANCH << "\n";
#endif

	unsigned int nLarge = 5;
	double diameterLarge = 10;
	double density = 0.1;
	double maxDisp = 0.1;

	double radiusLarge = 0.5*diameterLarge;
	//设置模拟框的周期性
	std::vector<bool> periodicity({ true,true });
	//计算出模拟盒的基本长度
	double baseLength = std::pow((M_PI*(nLarge*diameterLarge)) / (4.0*density), 1.0 / 2.0);
	std::vector<double> boxSize({ baseLength,baseLength });

	MersenneTwister rng;
	//初始化AABB树
	aabb::Tree treeLarge(2, maxDisp, periodicity, boxSize, nLarge);
	//初始化粒子位置向量
	std::vector<std::vector<double> > positionsLarge(nLarge, std::vector<double>(boxSize.size()));

	std::cout << "\nInserting large particles into AABB tree ...\n";
	for (unsigned int i = 0; i < nLarge; i++)
	{
		//初始化位置向量
		std::vector<double> position(2);
		//直接插入第一个粒子
		if (i == 0)
		{
			//生成一个随机粒子位置
			position[0] = boxSize[0] * rng();
			position[1] = boxSize[1] * rng();
		}
		//检查重叠
		else
		{
			//初始化重叠标记
			bool isOverlap = true;
			//继续尝试，直到没有重叠
			while (isOverlap)
			{
				//生成一个随机粒子位置
				position[0] = boxSize[0] * rng();
				position[1] = boxSize[1] * rng();
				//计算AABB下和上边界
				std::vector<double> lowerBound({ position[0] - radiusLarge,position[1] - radiusLarge });
				std::vector<double> upperBound({ position[0] - radiusLarge,position[1] + radiusLarge });
				//生成AABB
				aabb::AABB aabb(lowerBound, upperBound);
				//查询AABB重叠
				std::vector<unsigned int> particles = treeLarge.query(aabb);
				//标记为没有重叠
				isOverlap = false;
				//测试重叠
				for (unsigned int j = 0; j < particles.size(); j++)
				{
					//截止距离
					double cutOff = 2.0*radiusLarge;
					cutOff *= cutOff;
					//粒子重叠
					if (overlaps(position, positionsLarge[particles[j]], periodicity, boxSize, cutOff))
					{
						isOverlap = true;
						break;
					}
				}
			}
		}
		//将粒子插入到树中
		treeLarge.insertParticle(i, position, radiusLarge);
		//存储位置
		positionsLarge[i] = position;
	}
	std::cout << "Tree generated!\n";
}

bool overlaps(std::vector<double>& position1, std::vector<double>& position2,
	const std::vector<bool>& periodicity, const std::vector<double>& boxSize, double cutOff)
{
	// Calculate particle separation. 计算粒子分离
	std::vector<double> separation;
	separation.push_back(position1[0] - position2[0]);
	separation.push_back(position1[1] - position2[1]);

	// Calculate minimum image separation. 计算最小图像分离
	minimumImage(separation, periodicity, boxSize);

	double rSqd = separation[0] * separation[0] + separation[1] * separation[1];

	if (rSqd < cutOff) return true;
	else return false;
}

void minimumImage(std::vector<double>& separation,
	const std::vector<bool>& periodicity, const std::vector<double>& boxSize)
{
	for (unsigned int i = 0; i<2; i++)
	{
		if (separation[i] < -0.5*boxSize[i])
		{
			separation[i] += periodicity[i] * boxSize[i];
		}
		else
		{
			if (separation[i] >= 0.5*boxSize[i])
			{
				separation[i] -= periodicity[i] * boxSize[i];
			}
		}
	}
}

void periodicBoundaries(std::vector<double>& position,
	const std::vector<bool>& periodicity, const std::vector<double>& boxSize)
{
	for (unsigned int i = 0; i<2; i++)
	{
		if (position[i] < 0)
		{
			position[i] += periodicity[i] * boxSize[i];
		}
		else
		{
			if (position[i] >= boxSize[i])
			{
				position[i] -= periodicity[i] * boxSize[i];
			}
		}
	}
}
