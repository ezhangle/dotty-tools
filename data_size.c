#include <stdio.h>
#include <stdlib.h>
#include <float.h>

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
#if 0
	FILE * stat_fp = NULL;
#endif

	double x, y, z;
	double nx, ny, nz;

	double max_x, max_y, max_z;
	double min_x, min_y, min_z;

	unsigned long int num_points = 0;

	if( argc < 2 )
	{
		fprintf(stderr, "format is: %s <file-to-test> <opt: normalised-file>\n", argv[0] );
		return 1;
	}

	test_file = fopen( argv[1], "r");

	if( test_file==NULL)
	{
		printf("Error opening %s, aborting.\n", argv[1] );
		return EXIT_FAILURE;
	}

	if( EOF == fscanf(test_file, "%lf %lf %lf %lf %lf %lf", &x, &y, &z, &nx, &ny, &nz) )
	{
		fprintf(stderr, "Error reading file, must exit.\n");
		fclose(test_file);
		return EXIT_FAILURE;
	}

	max_x = min_x = x;
	max_y = min_y = y;
	max_z = min_z = z;
	++num_points;

#if 0
	stat_fp = fopen( "size_stats.gp", "w" );
	fprintf(stat_fp, "%lf %lf %lf\n", x, y, z);
#endif

	while( EOF != fscanf(test_file, " %lf %lf %lf %lf %lf %lf"
					, &x, &y, &z, &nx, &ny, &nz) )
	{
		max_x = max(x, max_x);
		max_y = max(y, max_y);
		max_z = max(z, max_z);

		min_x = min(x, min_x);
		min_y = min(y, min_y);
		min_z = min(z, min_z);

#if 0
		fprintf(stat_fp, "%lf %lf %lf\n", x, y, z);
#endif
		++num_points;
	}
#if 0
	fclose(stat_fp);
#endif
	
	
	normed_file = fopen( argv[2], "w");
	if( normed_file != NULL )
	{
		/* get a uniform range of point data */
		double avg_x = (max_x + min_x)/2.0;
		double avg_y = (max_y + min_y)/2.0;
		double avg_z = (max_z + min_z)/2.0;

		fseek(test_file, 0, SEEK_SET);
		while( EOF != fscanf(test_file, "%lf %lf %lf %lf %lf %lf"
				, &x, &y, &z, &nx, &ny, &nz) )
		{
			fprintf(normed_file, "%f %f %f %f %f %f\n"
				, x-avg_x, y-avg_y, z-avg_z, nx, ny, nz);
		}
		fclose(test_file);
		fclose(normed_file);
	}

	fprintf(stdout, "max_x = %f,\t min_x = %f\n", max_x, min_x);
	fprintf(stdout, "max_y = %f,\t min_y = %f\n", max_y, min_y);
	fprintf(stdout, "max_z = %f,\t min_z = %f\n", max_z, min_z);

	fprintf(stdout, "x_range = %f\n", max_x - min_x);
	fprintf(stdout, "y_range = %f\n", max_y - min_y);
	fprintf(stdout, "z_range = %f\n", max_z - min_z);

	return EXIT_SUCCESS;
}
