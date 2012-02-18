#include <stdio.h>
#include <stdlib.h>

void open_file(FILE **fp
		, char *filename
		, char *mode)
{
	*fp = fopen(filename, mode);

	if(*fp)
		return;

	fprintf(stderr, "Unable to open %s, aborting.\n", filename);
	exit(EXIT_FAILURE);
}

int detect_normals(FILE *fp)
{
	int items = 0;			/* items on the first line */
	int has_normals = 0;
	int ch;
	char *first_line = NULL;
	size_t line_length = 0;
	float fl;			/* dummy value for sscanf */

	do
	{
		ch = fgetc(fp);
		++line_length;
	} while(ch!=EOF && ch!='\n' && ch!='\r');

	if(line_length==0)
	{
		printf("Error reading first line of file.\n");
		exit(EXIT_FAILURE);
	}

	rewind(fp);
	first_line = malloc(line_length);

	if(NULL==first_line)
	{
		printf("Not enough memory for normal detection.\n");
		exit(EXIT_FAILURE);
	}

	fgets(first_line, (int)line_length, fp);
	rewind(fp);

	items = sscanf(first_line, "%f %f %f %f %f %f"
			, &fl , &fl , &fl , &fl , &fl , &fl);

	if(items == 6)
		has_normals = 1;
	else if(items == 3)
		has_normals = 0;
	else
	{
		printf("%d items present per line: ambiguous."
			" Unexpected results likely.\n", items);
		exit(EXIT_FAILURE);
	}

	free(first_line);
	return has_normals;
}
