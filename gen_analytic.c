#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "utilities.h"

void generate_plane(FILE *pc_fp, FILE *npc_fp);
void generate_torus(FILE *pc_fp, FILE *npc_fp);
void generate_cube(FILE *pc_fp, FILE *npc_fp);
void generate_sphere(FILE *pc_fp, FILE *npc_fp);
void generate_bulbous(FILE *pc_fp, FILE *npc_fp);
void generate_cone(FILE *pc_fp, FILE *npc_fp);

double principal_size;
double x, y, z, nx, ny, nz;

#define PI		3.14159265259
#define TWO_PI		(2*PI)
#define PI_BY_2		(PI/2.0)
#define PI_BY_4		(PI/4.0)

#define RANDY()		(( rand() / (RAND_MAX+1.0) ) - 0.5)

int main(int argc, char *argv[])
{
	FILE *pc_fp = NULL;
	FILE *npc_fp = NULL;

	if(argc != 2)
	{
		principal_size = 30.0;
	}
	else
	{
		principal_size = atof(argv[1]);
		printf("principal size = %f\n", principal_size);
	}

	open_file(&pc_fp, "plane.xyz", "w");
	open_file(&npc_fp, "nplane.xyz", "w");
	generate_plane(pc_fp, npc_fp);
	fclose(pc_fp);
	fclose(npc_fp);

	open_file(&pc_fp, "sphere.xyz", "w");
	open_file(&npc_fp, "nsphere.xyz", "w");
	generate_sphere(pc_fp, npc_fp);
	fclose(pc_fp);
	fclose(npc_fp);

#if 0
	open_file(&pc_fp, "torus.xyz", "w");
	open_file(&npc_fp, "ntorus.xyz", "w");
	generate_torus(pc_fp, npc_fp);
	fclose(pc_fp);
	fclose(npc_fp);
#endif

	open_file(&pc_fp, "cube.xyz", "w");
	open_file(&npc_fp, "ncube.xyz", "w");
	generate_cube(pc_fp, npc_fp);
	fclose(pc_fp);
	fclose(npc_fp);

#if 0
	open_file(&pc_fp, "cone.xyz", "w");
	open_file(&npc_fp, "ncone.xyz", "w");
	generate_cone(pc_fp, npc_fp);
	fclose(pc_fp);
	fclose(npc_fp);

	open_file(&pc_fp, "bulbous.xyz", "w");
	open_file(&npc_fp, "nbulbous.xyz", "w");
	generate_bulbous(pc_fp, npc_fp);
	fclose(pc_fp);
	fclose(npc_fp);
#endif

	return EXIT_SUCCESS;
}

void generate_plane(FILE *pc_fp, FILE *npc_fp)
{
	double length = principal_size;
	double width = principal_size;

	double lpos = -length/2.0;
	double wpos;

	for(; lpos < (length/2.0); lpos += 0.25)
	{
		for(wpos = -width/2.0; wpos < (width/2.0); wpos += 0.25)
		{
			fprintf(pc_fp, "%f %f %f\n", lpos, wpos, 0.0);

			fprintf(npc_fp, "%f %f %f ", lpos, wpos, 0.0);
			fprintf(npc_fp, "%f %f %f\n", 0.0, 0.0, 1.0);
		}
	}
	return;
}

void generate_sphere(FILE *pc_fp, FILE *npc_fp)
{
	double r = principal_size;
	double theta = -PI_BY_2;
	double psi = 0.0;
	double range = TWO_PI / 1000.0; 

	for(; theta < PI_BY_2; theta+=range)
	{
		double val = 1 + (500 * cos(theta));
		double inner_range = TWO_PI / val;
		for(psi = PI_BY_4; psi < PI_BY_4 + TWO_PI; psi+=inner_range)
		{
			nx = cos(theta) * cos(psi);
			ny = cos(theta) * sin(psi);
			nz = sin(theta);

			x = r * nx;
			y = r * ny;
			z = r * nz;

			fprintf(pc_fp, "%f %f %f\n", x, y, z);

			fprintf(npc_fp, "%f %f %f ", x, y, z);
			fprintf(npc_fp, "%f %f %f\n", nx, ny, nz);
		}
	}
	return;
}

/** r_1 = doughnut radius, r_2 = radius of ``tube'', r_2 assumed = 1.0 */
void generate_torus(FILE *pc_fp, FILE *npc_fp)
{
	double theta = 0.0;
	double psi = 0.0;
	double range = TWO_PI / 400.0; 

	double r_1 = principal_size;
	double r_2 = principal_size / 4.0;
	double pos[3];
	double nrm[3];
	double rm[3][3];

	int i=0;

	for(; theta < TWO_PI; theta+=range)
	{
		rm[0][0] = rm[1][1] = cos(theta);
		rm[0][1] = rm[1][0] = sin(theta);

		rm[0][2] = rm[1][2] = rm[2][0] = rm[2][1] = 0.0;
		rm[2][2] = 1.0;

		/* generate points on a circular slice of the torus */
		for(psi = 0.0; psi < TWO_PI; psi+=range)
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

			fprintf(pc_fp, "%f %f %f\n", pos[2], pos[1], pos[0]);

			fprintf(npc_fp, "%f %f %f ", pos[2], pos[1], pos[0]);
			fprintf(npc_fp, "%f %f %f\n", nrm[2], nrm[1], nrm[0]);
		}
	}

	return;
}	
void generate_cube(FILE *pc_fp, FILE *npc_fp)
{
	long int num_s = 400000;
	long int side_length = (long)principal_size;
	long int i = 0;

	for(; i != (num_s/6); ++i)
	{
		
		x = RANDY() * side_length;	nx = 0.0;
		y = RANDY() * side_length;	ny = 0.0;
		z = 0.5 * side_length;		nz = 1.0;

		fprintf(pc_fp, "%09.7f %09.7f %09.7f\n", x, y, z);
		fprintf(npc_fp, "%09.7f %09.7f %09.7f %09.7f %09.7f %09.7f\n"
				, x, y, z, nx, ny, nz);

		fprintf(pc_fp, "%09.7f %09.7f %09.7f\n", z, x, y);
		fprintf(npc_fp, "%09.7f %09.7f %09.7f %09.7f %09.7f %09.7f\n"
				, z, x, y, nz, nx, ny);

		fprintf(pc_fp, "%09.7f %09.7f %09.7f\n", x, z, y);
		fprintf(npc_fp, "%09.7f %09.7f %09.7f %09.7f %09.7f %09.7f\n"
				, x, z, y, nx, nz, ny);

		x *= -1.0; y *= -1.0; z*= -1.0;
		nx *= -1.0; ny *= -1.0; nz*= -1.0;

		fprintf(pc_fp, "%09.7f %09.7f %09.7f\n", x, y, z);
		fprintf(npc_fp, "%09.7f %09.7f %09.7f %09.7f %09.7f %09.7f\n"
				, x, y, z, nx, ny, nz);

		fprintf(pc_fp, "%09.7f %09.7f %09.7f\n", z, x, y);
		fprintf(npc_fp, "%09.7f %09.7f %09.7f %09.7f %09.7f %09.7f\n"
				, z, x, y, nz, nx, ny);

		fprintf(pc_fp, "%09.7f %09.7f %09.7f\n", x, z, y);
		fprintf(npc_fp, "%09.7f %09.7f %09.7f %09.7f %09.7f %09.7f\n"
				, x, z, y, nx, nz, ny);
	}
	return;
}
void generate_bulbous(FILE *pc_fp, FILE *npc_fp)
{
	double r;
	double range = TWO_PI / 3000.0; 

	double theta = 0.0, psi = 0.0;
	double dtheta, dpsi;

	for(; theta < PI; theta+=range)
	{
		/* generate points on a circular slice of the torus */
		for(psi = 0.0; psi < TWO_PI; psi+=range)
		{
			r = principal_size * sin(2*theta) * sin(2*psi);

			x = r * cos(theta) * cos(psi);
			y = r * sin(theta) * cos(psi);
			z = r * sin(psi);

			dtheta = 2 * cos(2*theta) * cos(2*psi) / r;
			dpsi = 2 * 2 * cos(theta) * cos(2*psi) / r;

			nx = r * cos(dtheta) * cos(dpsi);
			ny = r * sin(dtheta) * cos(dpsi);
			nz = r * sin(dpsi);
				
			fprintf(pc_fp, "%f %f %f\n", x, y, z);

			fprintf(npc_fp, "%f %f %f ", x, y, z);
			fprintf(npc_fp, "%f %f %f\n", nx, ny, nz);
		}
	}
	return;
}

void generate_cone(FILE *pc_fp, FILE *npc_fp)
{
	double base_radius = 30.0;
	double height = 100.0;

	double z = 0.0;
	double theta = 0.0;
	double phi;

	double radius = 0.0;
	double x, y;
	double nx, ny, nz;

	/* do the points on the base */
	for(theta = 0.0; theta < (2*PI); theta += 0.0005)
	{
		radius = base_radius * (rand() / (RAND_MAX + 1.0));
		x = radius * sin(theta);
		y = radius * cos(theta);
		
		fprintf(pc_fp, "%f %f %f\n", x, y, 0.0);

		fprintf(npc_fp, "%f %f %f ", x, y, 0.0);
		fprintf(npc_fp, "%f %f %f\n", 0.0, 0.0, -1.0);
	}

	/* do the points on the curved side */
	for(; z<height; z += 0.5)
	{
		double radius = base_radius * ((height - z) / height);

		double num_pts = 1+(5*(height - z));

		double phi = atan(base_radius / height);

		for(theta = 0.0; theta < (2*PI); theta += (2*PI/num_pts))
		{
			x = radius * sin(theta);
			y = radius * cos(theta);

			nx = sin(theta) * cos(phi);
			ny = cos(theta) * cos(phi);
			nz = sin(phi);

			if(z < 0.5)
				nz *= -1.0;

			fprintf(pc_fp, "%f %f %f\n", x, y, z);

			fprintf(npc_fp, "%f %f %f ", x, y, z);
			fprintf(npc_fp, "%f %f %f\n", nx, ny, nz);
		}
	}

	/* add a single point at the peak
	*	note that the normal is actually
	*	undefined at this point, but in practice
	*	can reasonably be taken as (0,0,1)
	*/
	fprintf(pc_fp, "%f %f %f\n", 0.0, 0.0, z);
	fprintf(npc_fp, "%f %f %f ", 0.0, 0.0, z);
	fprintf(npc_fp, "%f %f %f\n", 0.0, 0.0, 1.0);

	return;
}

