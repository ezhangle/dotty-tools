#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "utilities.h"

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

