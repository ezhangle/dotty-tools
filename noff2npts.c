#include <stdio.h>

/**
	This program reads a normal-augmented OFF file and
	produces a normal-augmented point cloud from the
	vertices.
*/
int main(int argc, char **argv)
{
	FILE *noff_file;
	FILE *npts_file;

	unsigned long int numverts, numfaces, numedges, i=0;

	float x, y, z, nx, ny, nz;

	if( argc != 3 )
	{
		printf("Syntax is: %s <noff-file> <npts_file>\n", argv[0] );
		return 1;
	}

	noff_file = fopen( argv[1], "r");
	npts_file = fopen( argv[2], "w");
	
	if(!noff_file || !npts_file)
	{
		if(noff_file)	fclose(noff_file);
		if(npts_file)	fclose(npts_file);

		fprintf(stderr, "Failed to open files.\n");
		return 1;
	}

	getc(noff_file);
	getc(noff_file);
	getc(noff_file);
	getc(noff_file);

	if( EOF == fscanf(noff_file, "%lu %lu %lu"
		, &numverts, &numfaces, &numedges) )
	{
		fprintf(stderr, "Error reading %s\n", argv[1]);
		fclose(npts_file);
		fclose(noff_file);
		return 1;
	}

	for(; i!=numverts; ++i)
	{
		if( EOF == fscanf(noff_file, "%f %f %f %f %f %f"
				, &x, &y, &z, &nx, &ny, &nz) )
		{
			fprintf(stderr, "Error reading %s\n", argv[1]);
			fclose(npts_file);
			fclose(noff_file);
			return 1;
		}

		if( 0 > fprintf(npts_file, "%f %f %f %f %f %f\n"
				, x, y, z, nx, ny, nz) )
		{
			fprintf(stderr, "Error writing to %s\n", argv[2]);
			fclose(npts_file);
			fclose(noff_file);
			return 1;
		}
	}

	fclose(noff_file);
	fclose(npts_file);

	return 0;
}
