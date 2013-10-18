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

int main(int argc, char *argv[])
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



typedef struct { double x, y, z; } vector;


int main(int argc, char *argv[])
{
	if(argc != 3)
	{
		std::cout << "You must specify 2 files" << std::endl;
		return 0;
	}

	pcl::PointCloud<pcl::PointXYZ> pt_cloud;
//	std::ifstream cloud(argv[1]);

	FILE *cloud = fopen(argv[1], "r");

	std::cerr << "prior to loop" << std::endl;

	int ctr = 0;
	double dummy;
//	while(!cloud.eof() && !cloud.ferror())
	do
	{
		pcl::PointXYZ pt;
		fscanf(cloud, "%f %f %f %f", &pt.x, &pt.y, &pt.z, &dummy);
		//cloud >> pt.x >> pt.y >> pt.z >> dummy;

		pt_cloud.push_back(pt);
	}
	while(!feof(cloud) && !ferror(cloud));
//	cloud.close();
	fclose(cloud);


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

const double PI = 3.14159265359;

typedef struct { double val, x, y, z; } eigenvector_st;

void read_data_files(char *argv[], eigenvector_st vectors[2][3]);

void rearrange_eigenvectors(eigenvector_st vectors[2][3]);

double compute_angle(eigenvector_st *A, eigenvector_st *B);

int sort_eigenvectors(const void *A, const void *B);


int main(int argc, char *argv[])
{
	eigenvector_st vectors[2][3];
	double angle[3] = { 0.0, 0.0, 0.0 };
	int i=0;

	if(argc != 3)
	{
		printf("syntax is: %s <file 1> <file 2>\n", argv[0]);
		exit(EXIT_FAILURE);
	}

	read_data_files(argv, vectors);
	rearrange_eigenvectors(vectors);

	for(i=0; i!=3; ++i)
	{
		angle[i] = compute_angle(&vectors[0][i], &vectors[1][i]);
		printf("angle[%d] = %f\n", i, angle[i]);
	}

	return 0;
}

void read_data_files(char *argv[], eigenvector_st vectors[2][3])
{
	int file = 0, i=0;

	for(file=0; file != 2; ++file)
	{
		FILE *data_fp = fopen(argv[file+1], "r");

		if(!data_fp)
		{
			printf("unable to open \"%s\"\n", argv[file+1]);
			exit(EXIT_FAILURE);
		}
		for(i=0; i!=3; ++i)
		{
			fscanf(data_fp, "%lf %lf %lf %lf\n"
				, &vectors[file][i].val
				, &vectors[file][i].x
				, &vectors[file][i].y
				, &vectors[file][i].z);
		}
		fclose(data_fp);
	}
	return;
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

double compute_angle(eigenvector_st *A, eigenvector_st *B)
{
	double size_A = sqrt( A->x*A->x + A->y*A->y + A->z*A->z);
	double size_B = sqrt( B->x*B->x + B->y*B->y + B->z*B->z);

	double dot_product = (A->x*B->x) + (A->y*B->y) + (A->z*B->z);
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

int main(int argc, char *argv[])
{
	int i = 0;
	evector A[3], B[3];
	evector rotations[3];

	FILE *eva_fp = NULL;
	FILE *evb_fp = NULL;
	FILE *ab_rotn_matrix = NULL;

	vector rotn_axes[3];

	if(argc != 4)
	{
		printf("usage: %s <evec file> <evec file> <output>\n"
			, argv[0]);
		return 0;
	}

	eva_fp = fopen(argv[1], "r");
	evb_fp = fopen(argv[2], "r");
	ab_rotn_matrix = fopen(argv[3], "w");

	if(!eva_fp || !evb_fp || !ab_rotn_matrix)
	{
		fprintf(stderr, "one or more files did not open\n");
		exit(EXIT_FAILURE);
	}
	

	/* read eigenvectors and eigenvalues from a file */
	for(; i!=Num_Evecs; ++i)
	{
		fscanf(eva_fp, "%lf %lf %lf %lf"
			, &A[i].eval
			, &A[i].evec.x
			, &A[i].evec.y
			, &A[i].evec.z);

		fscanf(evb_fp, "%lf %lf %lf %lf"
			, &B[i].eval
			, &B[i].evec.x
			, &B[i].evec.y
			, &B[i].evec.z);
	}

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

	for(i=0; i!=Num_Evecs; ++i)
	{
		fprintf(ab_rotn_matrix, "%f %f %f %f\n"
			, rotations[i].eval
			, rotations[i].evec.x
			, rotations[i].evec.y
			, rotations[i].evec.z);
	}

	fclose(eva_fp);
	fclose(evb_fp);
	fclose(ab_rotn_matrix);

	return 0;
}

