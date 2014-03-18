#include <stdio.h>
#include <stdlib.h>
#include <math.h>

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
		/*printf("angle[%d] = %f\n", i, angle[i]);*/
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


