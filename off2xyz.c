#include <stdio.h>

#include "utilities.h"

/**
	This program reads a normal-augmented OFF file and
	produces a normal-augmented point cloud from the
	vertices.
*/
int main(int argc, char **argv)
{
	FILE *off_file;
	FILE *xyz_file;

	unsigned long int numverts, numfaces, numedges, i = 0;

	float x, y, z, nx, ny, nz;

	if ( argc != 3 )
	{
		printf("Syntax is: %s <off-file> <xyz_file>\n", argv[0] );
		return 1;
	}

	open_file(&off_file, argv[1], "r");
	open_file(&xyz_file, argv[2], "w");

	getc(off_file);
	getc(off_file);
	getc(off_file);
	getc(off_file);

	if ( EOF == fscanf(off_file, "%lu %lu %lu"
					   , &numverts, &numfaces, &numedges) )
	{
		fprintf(stderr, "Error reading %s\n", argv[1]);
		fclose(xyz_file);
		fclose(off_file);
		return 1;
	}

	for (; i != numverts; ++i)
	{
		if ( EOF == fscanf(off_file, "%f %f %f %f %f %f"
						   , &x, &y, &z, &nx, &ny, &nz) )
		{
			fprintf(stderr, "Error reading %s\n", argv[1]);
			fclose(xyz_file);
			fclose(off_file);
			return 1;
		}

		if ( 0 > fprintf(xyz_file, "%f %f %f %f %f %f\n"
						 , x, y, z, nx, ny, nz) )
		{
			fprintf(stderr, "Error writing to %s\n", argv[2]);
			fclose(xyz_file);
			fclose(off_file);
			return 1;
		}
	}

	fclose(off_file);
	fclose(xyz_file);

	return 0;
}
