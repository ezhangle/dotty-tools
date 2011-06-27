#include <stdio.h>
#include <stdlib.h>
#include <math.h>

void generate_torus(FILE *sample_fp);
void generate_cube(FILE *sample_fp);
void generate_sphere(FILE *sample_fp);
void generate_bulbous(FILE *pcloud_fp);

double x, y, z, nx, ny, nz;

#define RANDY()		(( rand() / (RAND_MAX+1.0) ) - 0.5)

void abort_if_null(FILE *fp, char *filename)
{
	if(fp)
		return;

	fprintf(stderr, "Unable to open %s, aborting.\n", filename);
	exit(EXIT_FAILURE);
}

int main()
{
	FILE *pcloud_fp = NULL;

	pcloud_fp = fopen("sphere.npts", "w");
	abort_if_null(pcloud_fp, "sphere.npts");
	generate_sphere(pcloud_fp);
	fclose(pcloud_fp);

	pcloud_fp = fopen("torus.npts", "w");
	abort_if_null(pcloud_fp, "torus.npts");
	generate_torus(pcloud_fp);
	fclose(pcloud_fp);

	pcloud_fp = fopen("cube.npts", "w");
	abort_if_null(pcloud_fp, "cube.npts");
	generate_cube(pcloud_fp);
	fclose(pcloud_fp);

/*
	pcloud_fp = fopen("bulbous.npts", "w");
	abort_if_null(pcloud_fp, "bulbous.npts");
	generate_bulbous(pcloud_fp);
	fclose(pcloud_fp);
*/

	return EXIT_SUCCESS;
}

void generate_sphere(FILE *pcloud_fp)
{
	double r = 50;
	double theta = 0.0;
	double psi = 0.0;
	double range = 6.28318 / 1000.0; 

	for(; theta < 6.28318; theta+=range)
	{
		for(psi = 0.0; psi < 6.28318; psi+=range)
		{
			nx = cos(theta) * cos(psi);
			ny = cos(theta) * sin(psi);
			nz = sin(theta);

			x = r * nx;
			y = r * ny;
			z = r * nz;

			fprintf(pcloud_fp, "%f %f %f ", x, y, z);
			fprintf(pcloud_fp, "%f %f %f\n", nx, ny, nz);
		}
	}
	return;
}

/** r_1 = doughnut radius, r_2 = radius of ``tube'', r_2 assumed = 1.0 */
void generate_torus(FILE *pcloud_fp)
{
	double theta = 0.0;
	double psi = 0.0;
	double range = 6.28318 / 400.0; 

	double r_1 = 50.0;
	double r_2 = 15.0;
	double pos[3];
	double nrm[3];
	double rm[3][3];

	int i=0;

	for(; theta < 6.28318; theta+=range)
	{
		rm[0][0] = rm[1][1] = cos(theta);
		rm[0][1] = rm[1][0] = sin(theta);

		rm[0][2] = rm[1][2] = rm[2][0] = rm[2][1] = 0.0;
		rm[2][2] = 1.0;

		/* generate points on a circular slice of the torus */
		for(psi = 0.0; psi < 6.28318; psi+=range)
		{
			/* work in x/z plane for simplicity */
			x = r_1 + (r_2*cos(psi));
			y = ny = 0.0;
			z = r_2*sin(psi);

			nx = cos(psi);
			nz = sin(psi);

			/* rotate coordinates to generate entirety of torus */
			for(i=0; i!=3; ++i)
			{
				pos[i] = rm[i][0]*x + rm[i][1]*y + rm[i][2]*z;
				nrm[i] = rm[i][0]*nx + rm[i][1]*ny + rm[i][2]*nz;
			}

			fprintf(pcloud_fp, "%f %f %f ", pos[2], pos[1], pos[0]);
			fprintf(pcloud_fp, "%f %f %f\n", nrm[2], nrm[1], nrm[0]);
		}
	}

	return;
}	
void generate_cube(FILE *pcloud_fp)
{
	long int num_samples = 500000;
	long int side_length = 80;
	long int i = 0;

	for(; i != num_samples; ++i)
	{
		
		x = RANDY() * side_length;		nx = 0.0;
		y = RANDY() * side_length;		ny = 0.0;
		z = 0.5 * side_length;			nz = 1.0;
		fprintf(pcloud_fp, "%09.7f %09.7f %09.7f %09.7f %09.7f %09.7f\n"
				, x, y, z, nx, ny, nz);

		x = RANDY() * side_length;		nx = 0.0;
		y = RANDY() * side_length;		ny = 0.0;
		z = -0.5 * side_length;			nz = -1.0;
		fprintf(pcloud_fp, "%09.7f %09.7f %09.7f %09.7f %09.7f %09.7f\n"
				, x, y, z, nx, ny, nz);
		
		x = RANDY() * side_length;		nx = 0.0;
		y = 0.5 * side_length;			ny = 1.0;
		z = RANDY() * side_length;		nz = 0.0;
		fprintf(pcloud_fp, "%09.7f %09.7f %09.7f %09.7f %09.7f %09.7f\n"
				, x, y, z, nx, ny, nz);
		
		x = RANDY() * side_length;		nx = 0.0;
		y = -0.5 * side_length;			ny = -1.0;
		z = RANDY() * side_length;		nz = 0.0;
		fprintf(pcloud_fp, "%09.7f %09.7f %09.7f %09.7f %09.7f %09.7f\n"
				, x, y, z, nx, ny, nz);
		
		x = 0.5 * side_length;			nx = 1.0;
		y = RANDY() * side_length;		ny = 0.0;
		z = RANDY() * side_length;		nz = 0.0;
		fprintf(pcloud_fp, "%09.7f %09.7f %09.7f %09.7f %09.7f %09.7f\n"
				, x, y, z, nx, ny, nz);
		
		x = -0.5 * side_length;			nx = -1.0;
		y = RANDY() * side_length;		ny = 0.0;
		z = RANDY() * side_length;		nz = 0.0;
		fprintf(pcloud_fp, "%09.7f %09.7f %09.7f %09.7f %09.7f %09.7f\n"
				, x, y, z, nx, ny, nz);
	}
	return;
}
void generate_bulbous(FILE *pcloud_fp)
{
	double r = 40.0;
	double range = 6.28318 / 3000.0; 

	double theta = 0.0, psi = 0.0;
	double dtheta, dpsi;

	for(; theta < 3.141593; theta+=range)
	{
		/* generate points on a circular slice of the torus */
		for(psi = 0.0; psi < 6.28318; psi+=range)
		{
			r = 50.0 * sin(2*theta) * sin(2*psi);

			x = r * cos(theta) * cos(psi);
			y = r * sin(theta) * cos(psi);
			z = r * sin(psi);

			dtheta = 2 * cos(2*theta) * cos(2*psi) / r;
			dpsi = 2 * 2 * cos(theta) * cos(2*psi) / r;

			nx = r * cos(dtheta) * cos(dpsi);
			ny = r * sin(dtheta) * cos(dpsi);
			nz = r * sin(dpsi);
				
			fprintf(pcloud_fp, "%f %f %f ", x, y, z);
			fprintf(pcloud_fp, "%f %f %f\n", nx, ny, nz);
		}
	}
	return;
}
