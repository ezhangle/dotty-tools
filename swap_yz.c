#include <stdio.h>
#include <string.h>

/*
	This program swaps the y and z coordinates of a .npts file
*/

int main(int argc, char *argv[])
{
	FILE *infile;
	FILE *outfile;

	float x, y, z;
	float nx, ny, nz;

	enum {	XY_SWAP, XZ_SWAP, YZ_SWAP };
	int to_swap = XY_SWAP;

	if(argc!=4)
	{
		printf("Syntax is: %s <infile> <swapped-file> <\"xy\" swap>\n", argv[0]);
		return 0;
	}

	if(!strcmp("xy", argv[3]))
		to_swap = XY_SWAP;
	else if(!strcmp("xz", argv[3]))
		to_swap = XZ_SWAP;
	else if(!strcmp("yz", argv[3]))
		to_swap = YZ_SWAP;

	infile = fopen(argv[1], "r");
	if(infile == NULL)
	{
		printf("Error opening %s.", argv[1]);
		return 1;
	}

	outfile = fopen(argv[2], "w");
	if(outfile == NULL)
	{
		printf("Error opening %s.", argv[2]);
		return 1;
	}

	/* remember to swap the normals as well */
	switch(to_swap)
	{
	case XY_SWAP:
		while(EOF != fscanf(infile, "%f %f %f %f %f %f", &x, &y, &z, &nx, &ny, &nz))
			fprintf(outfile, "%f %f %f %f %f %f\n", y, x, z, ny, nx, nz);
		break;
	case XZ_SWAP:
		while(EOF != fscanf(infile, "%f %f %f %f %f %f", &x, &y, &z, &nx, &ny, &nz))
			fprintf(outfile, "%f %f %f %f %f %f\n", z, y, x, nz, ny, nx);
		break;

	case YZ_SWAP:
		while(EOF != fscanf(infile, "%f %f %f %f %f %f", &x, &y, &z, &nx, &ny, &nz))
			fprintf(outfile, "%f %f %f %f %f %f\n", x, z, y, nx, nz, ny);
		break;
	}

	fclose(infile);
	fclose(outfile);

	return 0;
}
