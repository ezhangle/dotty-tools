#include <stdio.h>
#include <stdlib.h>

#include "utilities.h"

int main(int argc, char *argv[])
{
	FILE *infile, *outfile;
	double shift_x, shift_y, shift_z;
	float x, y, z, nx, ny, nz;
	int has_normals;

	if(argc != 6)
	{
		printf("syntax is: %s <infile> <outfile>"
			" <xshift> <yshift> <zshift>\n", argv[0]);
		return 0;
	}

	shift_x = atof(argv[3]);
	shift_y = atof(argv[4]);
	shift_z = atof(argv[5]);

	open_file(&infile, argv[1], "r");
	open_file(&outfile, argv[2], "w");

	has_normals = detect_normals(infile);

	while(EOF != fscanf(infile, "%f %f %f", &x, &y, &z))
	{
		x += shift_x;
		y += shift_y;
		z += shift_z;

		fprintf(outfile, "%f %f %f\n", x, y, z);

		if(has_normals)
		{
			if(EOF == fscanf(infile, "%f %f %f", &nx, &ny, &nz))
				break;
			fprintf(outfile, "%f %f %f\n", nx, ny, nz);
		}
	}

	if(ferror(infile) || ferror(outfile))
	{
		printf("An error occurred, the output file is in"
			" an unknown state.\n");
		fclose(infile);
		fclose(outfile);
		return 1;
	}

	printf("Point cloud shifted by {x, y, z} = {%f, %f, %f}\n"
					, shift_x, shift_y, shift_z);

	fclose(infile);
	fclose(outfile);
	return 0;
}
