#include <stdio.h>
#include <stdlib.h>
#include <math.h>

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


