#include <stdio.h>

int detect_normals(FILE *fp);

void open_file(FILE **fp
		, char *filename
		, char *mode);

enum { X_Axis, Y_Axis, Z_Axis };

typedef struct { double x, y, z; } vector;
typedef struct { double eval; vector evec; } evector;

double angle( vector A, vector B );
vector cross_product(vector A, vector B);
int evec_comp(const void *one, const void *two);

void print_matrix(char *name, double mat[3][3]);
void rotate_vector(double rot[3][3], vector *new_vector);
void setup_for_rotation(double rot[3][3], int Axis, double theta);

void normalise_vector(vector *A);