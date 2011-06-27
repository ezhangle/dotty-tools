#include <stdio.h>
#include <stdlib.h>

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

	if( argc != 4 && argc !=5)
	{
		fprintf(stdout, "Syntax is: %s <scale-factor> <infile> "
				"<outfile> <opt: thinning>\n", argv[0] );
		return 1;
	}

	scale = atof( argv[1] );
	infile = fopen( argv[2], "r" );
	outfile = fopen( argv[3], "w" );

	if(argv[4])
		ratio = (unsigned long)atoi(argv[4]);

	if( infile == NULL )
	{
		fprintf(stderr, "Unable to open %s, exiting.\n", argv[2]);
		exit(EXIT_FAILURE);
	}

	if( outfile == NULL )
	{
		fprintf(stderr, "Unable to open %s, exiting.\n", argv[3]);
		exit(EXIT_FAILURE);
	}

	/* read until the file finishes */
	while( EOF != fscanf(infile, "%lf %lf %lf %lf %lf %lf", &x, &y, &z, &nx, &ny, &nz ) )
	{
		++counter;

		if( (counter % ratio) != 0)
			continue;

		fprintf(outfile, "%f %f %f ", scale*x, scale*y, scale*z);
		fprintf(outfile, "%f %f %f\n", nx, ny, nz );
	}

	(void)fclose(infile);
	(void)fclose(outfile);

	return EXIT_SUCCESS;
}
