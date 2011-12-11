#include <stdio.h>
#include <stdlib.h>

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
		return -1;
	}

	rewind(fp);
	first_line = malloc(line_length);

	if(NULL==first_line)
	{
		printf("Not enough memory for normal detection.\n");
		return -2;
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
		fprintf(stderr, "%d items present per line: ambiguous."
				" Unexpected results likely.\n", items);
		has_normals = -3;
	}

	return has_normals;
}
