/* This code is released under the GNU General Public Licence, version 3 */
/* A copy of this licence may be found at http://www.gnu.org/licenses/gpl.txt */

#include <stdio.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_eigen.h>
#include <gsl/gsl_statistics_double.h>

#include "pca.h"

int main(int argc, char *argv[])
{
        FILE * infile = NULL;
        FILE * outfile = NULL;

	double evecs[NUM_ROWS][NUM_COLS];
	double mean_x, mean_y, mean_z;
	unsigned long numpoints = 0;

	double * x_coords = NULL;
	double * y_coords = NULL;
	double * z_coords = NULL;

        if( argc != 3)
        {
                fprintf(stdout, "Syntax is: %s <infile> <outfile>\n", argv[0] );
                return 1;
        }

        infile = fopen( argv[1], "r" );
        outfile = fopen( argv[2], "w" );

        if( infile == NULL )
        {
                fprintf(stderr, "Unable to open %s, exiting.\n", argv[2]);
                exit(EXIT_FAILURE);
        }

        if( outfile == NULL )
        {
                fprintf(stderr, "Unable to open %s, exiting.\n", argv[3]);
                exit(EXIT_FAILURE);
        }

	compute_means(infile, &mean_x, &mean_y, &mean_z, &numpoints);

	x_coords = malloc(numpoints * sizeof(*x_coords));
	y_coords = malloc(numpoints * sizeof(*y_coords));
	z_coords = malloc(numpoints * sizeof(*z_coords));

	compute_PCA(infile, mean_x, mean_y, mean_z, numpoints, x_coords, y_coords, z_coords, evecs);
	transpose_matrix(evecs);
	normalise_matrix(evecs);
	write_transformed(infile, outfile, evecs, mean_x, mean_y, mean_z);

	fclose(infile);
	fclose(outfile);

	return 0;
}

void transpose_matrix(double matrix[NUM_ROWS][NUM_COLS])
{
	int i=0, j;
	double intermediate;

	for(; i!=NUM_ROWS; ++i)
	{
		for(j=0; j!=NUM_COLS; ++j)
		{
			intermediate = matrix[i][j];
			matrix[i][j] = matrix[j][i];
			matrix[j][i] = intermediate;
		}
	}

	return;
}

void normalise_matrix(double m[NUM_ROWS][NUM_COLS])
{
	int i=0, j;
	double det = 0.0;

	det += m[0][0] * (m[1][1]*m[2][2] - m[2][1]*m[1][2]);
	det -= m[0][1] * (m[1][0]*m[2][2] - m[2][0]*m[1][2]);
	det += m[0][2] * (m[1][0]*m[2][1] - m[2][0]*m[1][1]);

	for(; i!=NUM_ROWS; ++i)
	{
		for(j=0; j!=NUM_COLS; ++j)
			m[i][j] /= det;
	}

	return;
}

void write_transformed(FILE *infile
	, FILE *outfile
	, double evec[NUM_ROWS][NUM_COLS]
	, double mean_x
	, double mean_y
	, double mean_z)
{
	double x, y, z, tx, ty, tz;
	double nx, ny, nz, tnx, tny, tnz;

	vector new_mean;

	new_mean.x = evec[0][0]*mean_x + evec[0][1]*mean_y + evec[0][2]*mean_z;
	new_mean.y = evec[1][0]*mean_y + evec[1][1]*mean_y + evec[1][2]*mean_z;
	new_mean.z = evec[2][0]*mean_z + evec[2][1]*mean_y + evec[2][2]*mean_z;

        while( EOF != fscanf(infile, "%lf %lf %lf %lf %lf %lf"
					, &x, &y, &z, &nx, &ny, &nz) )
	{
		tx = x - mean_x;
		ty = y - mean_y;
		tz = z - mean_z;

		tnx = nx;
		tny = ny;
		tnz = nz;

		x = evec[0][0]*tx + evec[0][1]*ty + evec[0][2]*tz;
		y = evec[1][0]*tx + evec[1][1]*ty + evec[1][2]*tz;
		z = evec[2][0]*tx + evec[2][1]*ty + evec[2][2]*tz;

		nx = evec[0][0]*tnx + evec[0][1]*tny + evec[0][2]*tnz;
		ny = evec[1][0]*tnx + evec[1][1]*tny + evec[1][2]*tnz;
		nz = evec[2][0]*tnx + evec[2][1]*tny + evec[2][2]*tnz;

		fprintf(outfile, "%f %f %f %f %f %f\n", x, y, z, nx, ny, nz);
	}

	return;
}

void compute_means(FILE *infile
	, double *mean_x
	, double *mean_y
	, double *mean_z
	, unsigned long *pts)
{
	double mx=0.0, my=0.0, mz=0.0;
	double x, y, z;

	rewind(infile);

        while( EOF != fscanf(infile, "%lf %lf %lf %*f %*f %*f", &x, &y, &z) )
        {
		mx += x;
		my += y;
		mz += z;

		++*pts;
	}

	*mean_x = (mx / (double)*pts);
	*mean_y = (my / (double)*pts);
	*mean_z = (mz / (double)*pts);

	return;
}

void compute_PCA(FILE *infile
	, double mean_x
	, double mean_y
	, double mean_z
	, long int num_points
	, double *x_coords
	, double *y_coords
	, double *z_coords
	, double evecs[NUM_ROWS][NUM_COLS]) /* first index = component */
{
	double x, y, z;
	double nx, ny, nz;
	double xdiff, ydiff, zdiff;

	double cov_mat[NUM_ROWS][NUM_COLS];	/* covariance matrix */
	double evals[NUM_ROWS];			/* eigenvalues */
	/*double npts = (double)num_points;*/

	long int i=0;

	rewind(infile);
	for(i=0; i!=num_points; ++i)
	{
		fscanf(infile, " %lf %lf %lf %lf %lf %lf"
					, &x, &y, &z, &nx, &ny, &nz);
		x_coords[i] = x;
		y_coords[i] = y;
		z_coords[i] = z;

		xdiff = x - mean_x;
		ydiff = y - mean_y;
		zdiff = z - mean_z;
	}

	cov_mat[1][0] = cov_mat[0][1] = gsl_stats_covariance_m(x_coords, 1, y_coords, 1, num_points, mean_x, mean_y);
	cov_mat[2][0] = cov_mat[0][2] = gsl_stats_covariance_m(x_coords, 1, z_coords, 1, num_points, mean_x, mean_z);
	cov_mat[1][2] = cov_mat[2][1] = gsl_stats_covariance_m(y_coords, 1, z_coords, 1, num_points, mean_y, mean_z);

	cov_mat[0][0] = gsl_stats_variance_m(x_coords, 1, num_points, mean_x); 
	cov_mat[1][1] = gsl_stats_variance_m(y_coords, 1, num_points, mean_y); 
	cov_mat[2][2] = gsl_stats_variance_m(z_coords, 1, num_points, mean_z); 

	rewind(infile);

	get_eigenvectors(cov_mat, evals, evecs);

	return;
}

void normalise_evecs(double evecs[NUM_ROWS][NUM_COLS])
{
	int i=0;
	double xx; 
	double yy;
	double zz;
	double size;

	for(; i!=NUM_COLS; ++i)
	{
		xx = evecs[0][i] * evecs[0][i];
		yy = evecs[1][i] * evecs[1][i];
		zz = evecs[2][i] * evecs[2][i];

		size = sqrt(xx+yy+zz);
			
		evecs[0][i] /= size;
		evecs[1][i] /= size;
		evecs[2][i] /= size;
	}

	return;
}

void get_eigenvectors(double data_src[NUM_ROWS][NUM_COLS]
	, double evals[NUM_ROWS]
	, double evecs[NUM_ROWS][NUM_COLS])
{
	int i, j;
	double data[ NUM_ROWS*NUM_COLS ];

	gsl_matrix_view m;
	gsl_vector *eval;
	gsl_matrix *evec;
	gsl_eigen_symmv_workspace *w;

	for(i=0; i!=NUM_ROWS; ++i)
	{
		for(j=0; j!=NUM_COLS; ++j)
		{
			data[ (i*NUM_COLS) + j ] = data_src[i][j];
		}
	}

        m = gsl_matrix_view_array(data, NUM_ROWS, NUM_COLS);

        eval = gsl_vector_alloc(NUM_ROWS);

        evec = gsl_matrix_alloc(NUM_ROWS, NUM_COLS);

        w = gsl_eigen_symmv_alloc(3);
        
        gsl_eigen_symmv(&m.matrix, eval, evec, w);

        gsl_eigen_symmv_free(w);

        gsl_eigen_symmv_sort(eval, evec, GSL_EIGEN_SORT_ABS_ASC);
        
        for(i=0; i < NUM_ROWS; i++)
        {
		gsl_vector_view evec_i = gsl_matrix_column(evec, i);

		evals[i] = gsl_vector_get(eval, i);

		for(j=0; j!=NUM_COLS; ++j)
			evecs[j][i] = gsl_vector_get(&evec_i.vector, j);
        }

        gsl_vector_free(eval);
        gsl_matrix_free(evec);

	return;
}
