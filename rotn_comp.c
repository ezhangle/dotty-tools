#include <stdio.h>
#include <stdlib.h>
#include <math.h>

typedef struct { double x, y, z; } vector;
typedef struct { double eval; vector evec; } evector;

double angle( vector A, vector B );
vector cross_product(vector A, vector B);
int evec_comp(const void *one, const void *two);

const int Num_Evecs = 3;

int main(int argc, char *argv[])
{
	int i = 0;
	evector A[3], B[3];

	FILE *eva_fp = NULL;
	FILE *evb_fp = NULL;
	FILE *ab_rotn_matrix = NULL;

	vector rotn_axes[3];

	eva_fp = fopen(argv[1], "r");
	evb_fp = fopen(argv[2], "r");
	ab_rotn_matrix = fopen("output.rmx", "w");

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

		fprintf(ab_rotn_matrix, "%f %f %f %f\n"
			, angle(A[i].evec, B[i].evec)
			, rotn_axes[i].x
			, rotn_axes[i].y
			, rotn_axes[i].z);
	}

	fclose(eva_fp);
	fclose(evb_fp);
	fclose(ab_rotn_matrix);

	return 0;
}


vector cross_product(vector A, vector B)
{
	vector C;

	C.x =  (A.y * B.z) - (A.z * B.y);
	C.y = -(A.x * B.z) + (A.z * B.x);
	C.z =  (A.y * B.x) - (A.x * B.y);

	return C;
}

int evec_comp(const void *one, const void *two)
{
	evector *evec1 = (evector*)one;
	evector *evec2 = (evector*)two;

	if(evec1->eval > evec2->eval)
		return 1;

	if(evec1->eval < evec2->eval)
		return -1;

	return 0;
}


/* calculate angle between A and B using the dot product */
double angle( vector A, vector B )
{
	double dot_product = (double)A.x*B.x + A.y*B.y + A.z*B.z;

	double mod_A = sqrt( (double)A.x*A.x + A.y*A.y + A.z*A.z );
	double mod_B = sqrt( (double)B.x*B.x + B.y*B.y + B.z*B.z );

	return acos( dot_product/(mod_A * mod_B) );
}

