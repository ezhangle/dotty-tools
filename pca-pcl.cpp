#include <iostream>
#include <fstream>

#include <pcl/point_cloud.h>
#include <pcl/common/pca.h>
#include <pcl/impl/point_types.hpp>


#define NUM_COLS 3
#define NUM_ROWS 3

typedef struct { double x, y, z; } vector;


int main(int argc, char *argv[])
{
	if(argc != 3)
	{
		std::cout << "You must specify 2 files" << std::endl;
		return 0;
	}

	pcl::PointCloud<pcl::PointXYZ> pt_cloud;
	std::ifstream cloud(argv[1]);

	std::cerr << "prior to loop" << std::endl;

	int ctr = 0;
	double dummy;
	while(!cloud.eof())
	{
		pcl::PointXYZ pt;
		cloud >> pt.x >> pt.y >> pt.z >> dummy;

		pt_cloud.push_back(pt);
	}
	cloud.close();

	pcl::PCA<pcl::PointXYZ> pca;

	std::cerr << "setting input cloud" << std::endl;
	pca.setInputCloud(pt_cloud.makeShared());

	std::cerr << "getting stuff" << std::endl;
	Eigen::Matrix3f evecs = pca.getEigenVectors();
	Eigen::Vector3f evals = pca.getEigenValues();

	std::ofstream anglefile(argv[2]);
	for(int i=0; i!=NUM_ROWS; ++i)
	{
		anglefile << evals(i) << " ";
		anglefile << evecs(0,i) << " "
			<< evecs(1,i) << " "
			<< evecs(2,i) << std::endl;
	}
	anglefile.close();

	std::cerr << "done getting stuff" << std::endl;
	
	return 0;
}
