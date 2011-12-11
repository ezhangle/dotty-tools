#include <stdio.h>
#include <stdlib.h>

#include "utilities.h"

/**
*	This program will read a .bnpts file and write the corresponding
*	.npts (ASCII) file. The size of a float in the file MUST match the
*	size of a float in the compiler you are using, lest chaos ensue.
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

	open_file(&bin_fp, argv[1], "rb");
	open_file(&asc_fp, argv[2], "w");

	for(;;)
	{
		if(0 == fread(pos, sizeof(float), to_read, bin_fp)
			|| feof(bin_fp) )
		{
			break;
		}

		if( ferror(bin_fp) )
		{
			fprintf(stderr, "Error reading %s\n", argv[1]);
			fclose(bin_fp);
			fclose(asc_fp);
			return EXIT_FAILURE;
		}

		if(0 == fread(norm, sizeof(float), to_read, bin_fp)
			|| feof(bin_fp) )
		{
			break;
		}

 		if( ferror(bin_fp) )
		{
			fprintf(stderr, "Error reading %s, exiting.\n"
				, argv[1]);
			fclose(bin_fp);
			fclose(asc_fp);
			return EXIT_FAILURE;
		}

		if( 0 > fprintf(asc_fp, "%f %f %f %f %f %f\n"
				, pos[0], pos[1], pos[2]
				, norm[0], norm[1], norm[2] ) )
		{
			fprintf(stderr, "Error writing %s\n", argv[2]);
			fclose(bin_fp);
			fclose(asc_fp);
			return EXIT_FAILURE;
		}
	}

	fclose(bin_fp);
	fclose(asc_fp);

	return 0;
}
