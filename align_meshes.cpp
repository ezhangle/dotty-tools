#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <cmath>

#include <pcl/point_cloud.h>
#include <pcl/common/pca.h>
#include <pcl/impl/point_types.hpp>

#define NUM_COLS 3
#define NUM_ROWS 3

#include "utilities.h"

const double PI = 3.14159265359;

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
	
	perform_pca(cloudA, evecsA);
	perform_pca(cloudB, evecsB);

	find_evec_angles(argc, argv);
	compute_rotation_matrix(argc, argv);
	apply_rotation(argc, argv);

	return EXIT_SUCCESS;
}
	

int apply_rotation(int argc, char *argv[])
{
	double angle = 0.0;
	vector pt;

	/* rotation angle to put rot_axis into a plane */
	double planar_angle = 0.0;

	/* rotation angle to align rot_axis with standard axis */
	double axis_angle = 0.0;

	FILE *input_pts = NULL;
	FILE *output_pts = NULL;

	vector rot_axis = { 0.0, 0.0, 0.0 };
	vector temp_axis = rot_axis;

	double actual_mat[3][3];
	double planar_mat[3][3];
	double axis_mat[3][3];
	double inv_planar_mat[3][3];
	double inv_axis_mat[3][3];

	if(argc != 7)
	{
		printf("Format: %s <radians> <x-comp> <y-comp> <z-comp>"
			" <input> <output>\n"
			, argv[0]);
		return EXIT_FAILURE;
	}

	angle = atof(argv[1]);
	rot_axis.x = atof(argv[2]);
	rot_axis.y = atof(argv[3]);
	rot_axis.z = atof(argv[4]);

	input_pts = fopen(argv[5], "r");
	output_pts = fopen(argv[6], "w");

	printf("x = %f\ny = %f\nz = %f\n"
		, rot_axis.x
		, rot_axis.y
		, rot_axis.z);

	if(!input_pts || !output_pts)
	{
		printf("Unable to open files\n");
		exit(EXIT_FAILURE);
	}

	temp_axis = rot_axis;
	

	/* rotate around z axis such that x component is 0 */
	planar_angle = atan(-rot_axis.x / rot_axis.y);
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

	for(int i=0; i!=NUM_ROWS; ++i)
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


void rearrange_eigenvectors(evector vectors[2][3]);

double compute_angle(eigenvector *A, eigenvector *B);

int sort_eigenvectors(const void *A, const void *B);


int find_evec_angles(std::vector<double>& evec_angles
	, std::vector<evector> evecsA
	, std::vector<evector> evecsB)
{
	rearrange_eigenvectors(evecsA, evecsB);

	for(int i=0; i!=3; ++i)
	{
		double angle = compute_angle(evecsA[i], evecsB[i]));
		evec_angles.push_back(angle);
	}

	return 0;
}

void rearrange_eigenvectors(eigenvector_st vectors[2][3])
{
	int file = 0;

	for(file=0; file != 2; ++file)
	{
		qsort(&vectors[file][0], 3, sizeof(vectors[file][0]), sort_eigenvectors);
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

int sort_eigenvectors(const void *evec_A, const void *evec_B)
{
	eigenvector_st *A = (eigenvector_st*)evec_A;

	eigenvector_st *B = (eigenvector_st*)evec_B;

	if(A->val > B->val)
		return 1;

	if(A->val < B->val)
		return -1;

	return 0;
}

const int Num_Evecs = 3;

/*
*	This program computes the rotation axes from 2 eigenvector
*	files whose format is:
*		eigenvalue x-component y-component z-component
*	and prints them to a third file: "output.rmx".
*
*	Prior to any the eigenvectors are sorted in decreasing
*	order of (signed) eigenvalue size. The cross product is
*	then taken of the first pair of eigenvectors, then the
*	second, then the third. Since the cross product is not
*	commutative the first vector in "A x B" is taken from the
*	first file.
*/

int compute_rotation_matrix(std::vector<evector> evecsA
	, std::vector<evector> evecsB)
{
	evector rotations[3];

	FILE *ab_rotn_matrix = NULL;

	vector rotn_axes[3];

	ab_rotn_matrix = fopen(argv[3], "w");

	/* make sure the eigenvectors are sorted by size, 
	* in decreasing size */
	qsort(A, Num_Evecs, sizeof(evector), evec_comp);
	qsort(B, Num_Evecs, sizeof(evector), evec_comp);

	for(i=0; i!=Num_Evecs; ++i)
	{
		rotn_axes[i] = cross_product(A[i].evec, B[i].evec);

		rotations[i].eval = angle(A[i].evec, B[i].evec);
		rotations[i].evec = rotn_axes[i];
	}
	/*qsort(rotations, Num_Evecs, sizeof(evector), evec_comp);*/

	for(int i=0; i!=Num_Evecs; ++i)
	{
		fprintf(ab_rotn_matrix, "%f %f %f %f\n"
			, rotations[i].eval
			, rotations[i].evec.x
			, rotations[i].evec.y
			, rotations[i].evec.z);
	}

	fclose(ab_rotn_matrix);

	return 0;
}

