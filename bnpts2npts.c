#include <stdio.h>
#include <stdlib.h>

/**
*	This program will read a .bnpts file and write the corresponding .npts (ASCII) file
*	The size of a float in the file MUST match the size of a float in the compiler you
*	are using, lest chaos ensue.
*/
int main(int argc, char **argv)
{
	FILE *bin_fp;
	FILE *asc_fp;

	float pos[3] = {0.0, 0.0, 0.0 };
	float norm[3] = {0.0, 0.0, 0.0 };

	size_t to_read = (size_t)3;

	if( argc != 3 )
	{
		printf("Syntax is: %s <input bnpts> <output npts>\n", argv[0] );
		return EXIT_SUCCESS;
	}

	bin_fp = fopen( argv[1], "rb" );
	if( bin_fp==NULL )
	{
		return EXIT_FAILURE;
	}

	asc_fp = fopen( argv[2], "w" );
	if( asc_fp == NULL )
	{
		fclose(bin_fp);
		return EXIT_FAILURE;
	}

	for(;;)
	{
		if( 0 == fread(pos, sizeof(float), to_read, bin_fp) || feof(bin_fp) )
			break;

		if( ferror(bin_fp) )
		{
			fprintf(stderr, "Error reading .bnpts file, exiting.\n");
			fclose(bin_fp);
			fclose(asc_fp);
			return EXIT_FAILURE;
		}

		if( 0 == fread(norm, sizeof(float), to_read, bin_fp) || feof(bin_fp) )
			break;

 		if( ferror(bin_fp) )
		{
			fprintf(stderr, "Error reading .bnpts file, exiting.\n");
			fclose(bin_fp);
			fclose(asc_fp);
			return EXIT_FAILURE;
		}

		if( 0 > fprintf(asc_fp, "%f %f %f %f %f %f\n"
				, pos[0], pos[1], pos[2], norm[0], norm[1], norm[2] ) )
		{
			fprintf(stderr, "Error writing .npts file, exiting.\n");
			fclose(bin_fp);
			fclose(asc_fp);
			return EXIT_FAILURE;
		}
	}

	printf("File converted.\n");
	fclose(bin_fp);
	fclose(asc_fp);

	return 0;
}
