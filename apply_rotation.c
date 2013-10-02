#include <stdio.h>
#include <stdlib.h>
#include <math.h>

typedef struct { double x, y, z; } vector;

void rotate_vector(double rot[3][3], vector *new_vector);
void setup_for_rotation(double rot[3][3], int Axis, double theta);
void print_matrix(char *name, double mat[3][3]);


enum
{
	X_Axis,
	Y_Axis,
	Z_Axis
};

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

#if 0
	print_matrix("planar", planar_mat);
	print_matrix("axis", axis_mat);
	print_matrix("actual", actual_mat);
	print_matrix("inv_axis", inv_axis_mat);
	print_matrix("inv_planar", inv_planar_mat);
#endif

	return EXIT_SUCCESS;
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






