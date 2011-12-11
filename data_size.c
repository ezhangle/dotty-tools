#include <stdio.h>
#include <stdlib.h>
#include <float.h>

#include "utilities.h"
#include "macros.h"

/**
	this program reads a normal-augmented point cloud
	in "x y z nx ny nz" format and prints the extension
	of the point cloud in the x, y and z directions
*/
int main(int argc, char **argv)
{
	FILE * test_file = NULL;
	FILE * normed_file = NULL;

	double x, y, z;
	double nx, ny, nz;

	double max_x, max_y, max_z;
	double min_x, min_y, min_z;

	unsigned long int num_points = 0;
	int has_normals = 0;

	char *format_string = NULL;

	if( argc < 2 )
	{
		fprintf(stderr, "format is: %s <file-to-test>"
				" <opt: normalised-file>\n", argv[0] );
		return 1;
	}

	test_file = fopen( argv[1], "r");
	if( test_file==NULL)
	{
		printf("Error opening %s, aborting.\n", argv[1] );
		return EXIT_FAILURE;
	}

	if(argc == 3)
	{
		normed_file = fopen(argv[2], "w");
		printf("Error opening %s, aborting.\n", argv[2] );
		fclose(test_file);
		return EXIT_FAILURE;
	}

	has_normals = detect_normals(test_file);

	switch(has_normals)
	{
		case -3:
		case -2:
		case -1:
			fclose(test_file);
			exit(EXIT_FAILURE);
		case 0:
			format_string = "%lf %lf %lf\n";
			printf("no normals found\n");
			break;
		case 1: 
			format_string = "%lf %lf %lf %*f %*f %*f\n";
			printf("normals detected\n");
			break;
	}

	if( EOF == fscanf(test_file, format_string, &x, &y, &z) )
	{
		fprintf(stderr, "Error reading file, must exit.\n");
		fclose(test_file);
		return EXIT_FAILURE;
	}

	max_x = min_x = x;
	max_y = min_y = y;
	max_z = min_z = z;
	++num_points;

	while( EOF != fscanf(test_file, format_string, &x, &y, &z) )
	{
		max_x = max(x, max_x);
		max_y = max(y, max_y);
		max_z = max(z, max_z);

		min_x = min(x, min_x);
		min_y = min(y, min_y);
		min_z = min(z, min_z);

		++num_points;
	}

	/* will only be null here if no argument provided */
	if(normed_file != NULL)
	{
		/* get a uniform range of point data */
		double mid_x = (max_x + min_x)/2.0;
		double mid_y = (max_y + min_y)/2.0;
		double mid_z = (max_z + min_z)/2.0;

		fseek(test_file, 0, SEEK_SET);

		/* if there are no normals, nx, ny and nz are ignored */
		while( EOF != fscanf(test_file, format_string
				, &x, &y, &z, &nx, &ny, &nz) )
		{
			fprintf(normed_file, "%f %f %f"
				, x-mid_x, y-mid_y, z-mid_z);

			if(has_normals)
			{
				fprintf(normed_file, " %f %f %f"
							, nx, ny, nz);
			}
			fprintf(normed_file, "\n");
		}
		fclose(normed_file);
	}

	printf("max_x = %f,\t min_x = %f\n", max_x, min_x);
	printf("max_y = %f,\t min_y = %f\n", max_y, min_y);
	printf("max_z = %f,\t min_z = %f\n", max_z, min_z);

	printf("x_range = %f\n", max_x - min_x);
	printf("y_range = %f\n", max_y - min_y);
	printf("z_range = %f\n", max_z - min_z);

	fclose(test_file);

	return EXIT_SUCCESS;
}
