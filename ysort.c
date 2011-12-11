#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "utilities.h"

typedef struct { float x, y, z; }	vector;
typedef struct { vector pos, norm; }	sample_point;

int cmp(const void *arg1, const void *arg2);

void store_samples(FILE *fp
	, sample_point *samples
	, unsigned long count);

int main(int argc, char *argv[])
{
	double last_z;			/* z coord of last dumping */

	sample_point pt;

	/* used for storage, realloc */
	sample_point *samples = NULL;
	sample_point *tmp = NULL;

	unsigned long count = 0;

	FILE *samp_fp = NULL;
	FILE *ysort_fp = NULL;

	if(argc != 3)
	{
		fprintf(stderr, "usage: %s <infile> <outfile>\n", argv[0]);
		exit(EXIT_FAILURE);
	}

	samp_fp = fopen(argv[1], "r");
	ysort_fp = fopen(argv[2], "w");

	open_file(&samp_fp, argv[1], "r");
	open_file(&ysort_fp, argv[2], "w");

	fscanf(samp_fp, "%*f %*f %lf", &last_z);
	rewind(samp_fp);

	while(EOF != fscanf(samp_fp, "%f %f %f %f %f %f"
		, &pt.pos.x, &pt.pos.y, &pt.pos.z
		, &pt.norm.x, &pt.norm.y, &pt.norm.z))
	{
		tmp = realloc(samples, (count+1)*sizeof(*samples));

		if(tmp==NULL)
		{
			free(samples);
			fclose(samp_fp);
			fprintf(stderr, "Unable to allocate memory.\n");
			exit(EXIT_FAILURE);
		}

		samples = tmp;

		samples[count] = pt;

		++count;

		if( fabs(pt.pos.z - last_z) > 1.0)
		{
			qsort(samples, count, sizeof(*samples), cmp);
			store_samples(ysort_fp, samples, count);

			free(samples);
			samples = NULL;
			last_z = pt.pos.z;
			count = 0L;
		}
	}

	if(!samples)
	{
		fclose(samp_fp);
		fclose(ysort_fp);
		fprintf(stderr, "no samples found.\n");
		exit(EXIT_FAILURE);
	}
	
	qsort(samples, count, sizeof(*samples), cmp);
	store_samples(ysort_fp, samples, count);

	fclose(samp_fp);
	fclose(ysort_fp);
	free(samples);

	return 0;
}


int cmp(const void *arg1, const void *arg2)
{
	sample_point *pt1 = (sample_point*)arg1;
	sample_point *pt2 = (sample_point*)arg2;

	if(pt1->pos.y < pt2->pos.y)
		return -1;
	else if(pt1->pos.y > pt2->pos.y)
		return 1;

	return 0;
}

void store_samples(FILE *fp, sample_point *samples, unsigned long count)
{
	unsigned long i = 0L;

	for(; i!=count; ++i)
	{
		fprintf(fp, "%f %f %f %f %f %f\n"
			, samples[i].pos.x, samples[i].pos.y
			, samples[i].pos.z, samples[i].norm.x
			, samples[i].norm.y, samples[i].norm.z);
	}

	return;
}
