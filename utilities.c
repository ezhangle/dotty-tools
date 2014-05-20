#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>

#include "utilities.h"

void open_file(FILE **fp
		, char *filename
		, char *mode)
{
	*fp = fopen(filename, mode);

	if(*fp)
		return;

	fprintf(stderr, "Unable to open %s, aborting.\n", filename);
	exit(EXIT_FAILURE);
}

void normalise_vector(vector *A)
{
	double xx = A->x * A->x;
	double yy = A->y * A->y;
	double zz = A->z * A->z;

	double size = (double)sqrt(xx + yy + zz);

	if(size > DBL_EPSILON)
	{
		A->x /= size;
		A->y /= size;
		A->z /= size;
	}
	
	return;
}

int detect_normals(FILE *fp)
{
	int items = 0;			/* items on the first line */
	int has_normals = 0;
	int ch;
	char *first_line = NULL;
	size_t line_length = 0;
	float fl;			/* dummy value for sscanf */

	do
	{
		ch = fgetc(fp);
		++line_length;
	} while(ch!=EOF && ch!='\n' && ch!='\r');

	if(line_length==0)
	{
		printf("Error reading first line of file.\n");
		exit(EXIT_FAILURE);
	}

	rewind(fp);
	first_line = malloc(line_length);

	if(NULL==first_line)
	{
		printf("Not enough memory for normal detection.\n");
		exit(EXIT_FAILURE);
	}

	fgets(first_line, (int)line_length, fp);
	rewind(fp);

	items = sscanf(first_line, "%f %f %f %f %f %f"
			, &fl , &fl , &fl , &fl , &fl , &fl);

	if(items == 6)
		has_normals = 1;
	else if(items == 3)
		has_normals = 0;
	else
	{
		printf("%d items present per line: ambiguous."
			" Unexpected results likely.\n", items);
		exit(EXIT_FAILURE);
	}

	free(first_line);
	return has_normals;
}

void print_matrix(char *name, double mat[3][3])
{
	printf("Matrix \"%s\":\n", name);
	printf("%06.5f %06.5f %06.5f\n", mat[0][0], mat[0][1], mat[0][2]); 
	printf("%06.5f %06.5f %06.5f\n", mat[1][0], mat[1][1], mat[1][2]); 
	printf("%06.5f %06.5f %06.5f\n", mat[2][0], mat[2][1], mat[2][2]); 
	printf("\n");
}

void rotate_vector(double rot[3][3], vector *new_vector)
{
	vector vect = *new_vector;

	new_vector->x =    (rot[0][0] * vect.x)
			+ (rot[0][1] * vect.y)
			+ (rot[0][2] * vect.z);
	
	new_vector->y =    (rot[1][0] * vect.x)
			+ (rot[1][1] * vect.y)
			+ (rot[1][2] * vect.z);

	new_vector->z =    (rot[2][0] * vect.x)
			+ (rot[2][1] * vect.y)
			+ (rot[2][2] * vect.z);

	return;
}

void setup_for_rotation(double rot[3][3], int Axis, double theta)
{
	switch(Axis)
	{
		case X_Axis:
			rot[0][0] = 1.0;
			rot[0][1] = 0.0;
			rot[0][2] = 0.0;

			rot[1][0] = 0.0;
			rot[1][1] = cos(theta);
			rot[1][2] = sin(theta);

			rot[2][0] = 0.0;
			rot[2][1] = -sin(theta);
			rot[2][2] = cos(theta);
			return;

		case Y_Axis:
			rot[0][0] = cos(theta);
			rot[0][1] = 0.0;
			rot[0][2] = sin(theta);

			rot[1][0] = 0.0;
			rot[1][1] = 1.0;
			rot[1][2] = 0.0;

			rot[2][0] = -sin(theta);
			rot[2][1] = 0.0;
			rot[2][2] = cos(theta);
			break;

		case Z_Axis:
			rot[0][0] = cos(theta);
			rot[0][1] = sin(theta);
			rot[0][2] = 0.0;

			rot[1][0] = -sin(theta);
			rot[1][1] = cos(theta);
			rot[1][2] = 0.0;

			rot[2][0] = 0.0;
			rot[2][1] = 0.0;
			rot[2][2] = 1.0;
			break;
	}
	return;
}

vector cross_product(vector A, vector B)
{
	vector C;

	C.x =  (A.y * B.z) - (A.z * B.y);
	C.y = -(A.x * B.z) + (A.z * B.x);
	C.z =  (A.x * B.y) - (A.y * B.x);

	return C;
}

int evec_comp(const void *one, const void *two)
{
	evector *evec1 = (evector*)one;
	evector *evec2 = (evector*)two;

	if(fabs(evec1->eval) > fabs(evec2->eval))
		return 1;

	if(fabs(evec1->eval) < fabs(evec2->eval))
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





