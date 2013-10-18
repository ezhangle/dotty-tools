#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <cmath>

#include <pcl/point_cloud.h>
#include <pcl/common/pca.h>
#include <pcl/impl/point_types.hpp>

/* utilities.h is used for C programs as well */
extern "C" {
	#include "utilities.h"
}

const double PI = 3.14159265359;
const int Num_Rows = 3;

void perform_pca(FILE *cloud, std::vector<evector>& eigenvectors);

int apply_rotation(int argc, char *argv[]);

int find_evec_angles(std::vector<double>& evec_angles
	, std::vector<evector> evecsA
	, std::vector<evector> evecsB);

int compute_rotations(std::vector<evector> evecsA
	, std::vector<evector> evecsB);

void sort_eigenvectors(std::vector<evector>& evecs);

double compute_angle(evector &A, evector &B);

/* same as an eigenvector, but with clearer variable names */
typedef struct { double angle; vector axis; } rotation;



int main(int argc, char *argv[])
{
	if(argc != 5)
	{
		std::cout << "syntax: " << argv[0];
		std::cout << " <cloud A> <cloud B> <mesh B>";
		std::cout << " <rotated mesh B>" << std::endl;

		return EXIT_SUCCESS;
	}
	FILE *cloudA = fopen(argv[1], "r");
	FILE *cloudB = fopen(argv[2], "r");
	FILE *input_mesh = fopen(argv[3], "r");
	FILE *output_mesh = fopen(argv[4], "w");

	std::vector<evector> evecsA, evecsB;
	std::vector<double> evec_angles;
	std::vector<rotation> rotns;
	
	perform_pca(cloudA, evecsA);
	perform_pca(cloudB, evecsB);

	find_evec_angles(evec_angles, evecsA, evecsB);
	compute_rotations(rotns, evecsA, evecsB);

	rotate_vector(rotns1




	apply_rotation(rotns[0], input_mesh, output_mesh);

	return EXIT_SUCCESS;
}
	

int apply_rotation(rotation &rotn
	, FILE *input_mesh
	, FILE *output_mesh)
{
	double angle = 0.0;
	vector pt;

	/* rotation angle to put rot_axis into a plane */
	double planar_angle = 0.0;

	/* rotation angle to align rot_axis with standard axis */
	double axis_angle = rotns

	double actual_mat[3][3];
	double planar_mat[3][3];
	double axis_mat[3][3];
	double inv_planar_mat[3][3];
	double inv_axis_mat[3][3];

	angle = atof(argv[1]);
	rot_axis.x = atof(argv[2]);
	rot_axis.y = atof(argv[3]);
	rot_axis.z = atof(argv[4]);

	input_pts = fopen(argv[5], "r");
	output_pts = fopen(argv[6], "w");

	printf("x = %f\ny = %f\nz = %f\n"
		, rotn.axis.x
		, rotn.axis.y
		, rotn.axis.z);

	temp_axis = rotn.axis;
	

	/* rotate around z axis such that x component is 0 */
	planar_angle = atan(-rotn.axis.x / rotn.axis.y);
	setup_for_rotation(planar_mat, Z_Axis, planar_angle);
	setup_for_rotation(inv_planar_mat, Z_Axis, -planar_angle);
	rotate_vector(planar_mat, &temp_axis);

	printf("YZ contained:\nx: %f\ny: %f\nz = %f\n"
		, temp_axis.x, temp_axis.y, temp_axis.z);

	setup_for_rotation(actual_mat, Y_Axis, angle);

	/* rotate around X axis such that z component is 0 */
	axis_angle = atan(temp_axis.z / temp_axis.y);
	setup_for_rotation(axis_mat, X_Axis, axis_angle);
	setup_for_rotation(inv_axis_mat, X_Axis, -axis_angle);
	rotate_vector(axis_mat, &temp_axis);

	printf("Y parallel:\nx: %f\ny: %f\nz = %f\n"
		, temp_axis.x, temp_axis.y, temp_axis.z);
	
	while(EOF != fscanf(input_pts, "%lf %lf %lf %*f", &pt.x, &pt.y, &pt.z))
	{
		rotate_vector(planar_mat, &pt);
		rotate_vector(axis_mat, &pt);
		rotate_vector(actual_mat, &pt);
		rotate_vector(inv_axis_mat, &pt);
		rotate_vector(inv_planar_mat, &pt);

		fprintf(output_pts, "%f %f %f 1.0\n", pt.x, pt.y, pt.z);
	}

	return EXIT_SUCCESS;
}


void perform_pca(FILE *cloud, std::vector<evector>& eigenvectors)
{
	pcl::PointCloud<pcl::PointXYZ> pt_cloud;

	int ctr = 0;
	double dummy;
	do
	{
		pcl::PointXYZ pt;
		fscanf(cloud, "%f %f %f %f", &pt.x, &pt.y, &pt.z, &dummy);

		pt_cloud.push_back(pt);
	}
	while(!feof(cloud) && !ferror(cloud));
	fclose(cloud);


	pcl::PCA<pcl::PointXYZ> pca;

	std::cerr << "setting input cloud" << std::endl;
	pca.setInputCloud(pt_cloud.makeShared());

	std::cerr << "getting stuff" << std::endl;
	Eigen::Matrix3f evecs = pca.getEigenVectors();
	Eigen::Vector3f evals = pca.getEigenValues();

	for(int i=0; i!=Num_Rows; ++i)
	{
		evector tmp;

		tmp.eval = evals(i);
		tmp.evec.x = evecs(0,i);
		tmp.evec.x = evecs(1,i);
		tmp.evec.x = evecs(2,i);

		eigenvectors.push_back(tmp);
	}
	return;
}


int find_evec_angles(std::vector<double>& evec_angles
	, std::vector<evector> evecsA
	, std::vector<evector> evecsB)
{
	sort_eigenvectors(evecsA);
	sort_eigenvectors(evecsB);

	for(int i=0; i!=3; ++i)
	{
		double angle = compute_angle(evecsA[i], evecsB[i]);
		evec_angles.push_back(angle);
	}

	return 0;
}

void sort_eigenvectors(std::vector<evector>& evecs)
{
	if(evecs[0].eval > evecs[1].eval)
	{
		evector tmp = evecs[1];
		evecs[1] = evecs[0];
		evecs[0] = tmp;
	}

	if(evecs[1].eval > evecs[2].eval)
	{
		evector tmp = evecs[2];
		evecs[2] = evecs[1];
		evecs[1] = tmp;
	}

	if(evecs[0].eval > evecs[1].eval)
	{
		evector tmp = evecs[1];
		evecs[1] = evecs[0];
		evecs[0] = tmp;
	}

	return;
}

double compute_angle(evector &A, evector &B)
{
	double size_A = sqrt( A.evec.x*A.evec.x
			+ A.evec.y*A.evec.y
			+ A.evec.z*A.evec.z);

	double size_B = sqrt( B.evec.x*B.evec.x
			+ B.evec.y*B.evec.y
			+ B.evec.z*B.evec.z);

	double dot_product = (A.evec.x*B.evec.x)
				+ (A.evec.y*B.evec.y)
				+ (A.evec.z*B.evec.z);

	double radians = acos(dot_product / (size_A*size_B)) ;

	return radians;
}

void compute_rotations(std::vector<rotation>& rotns
	, std::vector<evector> evecsA
	, std::vector<evector> evecsB)
{
	sort_eigenvectors(evecsA);
	sort_eigenvectors(evecsB);

	const int Num_Evecs = 3;
	for(int i=0; i!=Num_Evecs; ++i)
	{
		rotns[i].angle = angle(evecsA[i].evec, evecsB[i].evec);
		rotns[i].axis = cross_product(evecsA[i].evec, evecsB[i].evec);
	}

	return;
}

