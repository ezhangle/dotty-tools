#include <stdio.h>
#include <stdlib.h>

#include "utilities.h"
#include "macros.h"

int main(int argc, char **argv)
{
	FILE * infile = NULL;
	FILE * outfile = NULL;

	double scale;
	double x, y, z;
	double nx, ny, nz;

	unsigned long ratio = 1;
	unsigned long counter = 0;
	int has_normals = 0;
	char *format_string = NULL;

	if( argc != 4 && argc !=5)
	{
		fprintf(stdout, "Syntax is: %s <scale-factor> <infile> "
				"<outfile> <opt: thinning>\n", argv[0] );
		return 1;
	}

	scale = atof( argv[1] );
	open_file(&infile, argv[2], "r");
	open_file(&outfile, argv[3], "w");

	if(argv[4])
		ratio = (unsigned long)atoi(argv[4]);

	has_normals = detect_normals(infile);
	if(has_normals)
		format_string = "%lf %lf %lf";
	else
		format_string = "%lf %lf %lf %lf %lf %lf";

	/* if there are no normals, nx, ny and nz are ignored */
	while( EOF != fscanf(infile, format_string
				, &x, &y, &z, &nx, &ny, &nz ) )
	{
		++counter;

		if( (counter % ratio) != 0)
			continue;

		fprintf(outfile, "%f %f %f ", scale*x, scale*y, scale*z);
		if(has_normals)
			fprintf(outfile, " %f %f %f", nx, ny, nz );
		fprintf(outfile, "\n");
	}

	fclose(infile);
	fclose(outfile);

	return EXIT_SUCCESS;
}
