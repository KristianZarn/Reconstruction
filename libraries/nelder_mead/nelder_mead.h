#ifndef NELDER_MEAD_H
#define NELDER_MEAD_H

//-----------------------------------------------------------------------------
// Definitions
//-----------------------------------------------------------------------------

#define PI 3.1415926535897932384626433832795

#define RHO 1.0
#define CHI 2.0
#define GAMMA 0.5
#define SIGMA 0.5

// define a generic point containing a position (x) and a value (fx)
typedef struct {
  double *x;
  double fx;
} point_t;

// define a simplex struct containing an array of n+1 points (p)
// each having dimension (n)
typedef struct {
  point_t *p;
  int n;
} simplex_t;

// define optimization settings
typedef struct {
  double tolx;
  double tolf;
  int max_iter;
  int max_eval;
  int verbose;
} optimset_t;

//-----------------------------------------------------------------------------
// Cost function interface
//-----------------------------------------------------------------------------

typedef void (*fun_t)(int, point_t *, const void *);

//-----------------------------------------------------------------------------
// Nelder-Mead algorithm and template cost function
//-----------------------------------------------------------------------------
void nelder_mead(int, const point_t *, point_t *, fun_t, const void *, const optimset_t *);

//-----------------------------------------------------------------------------
// Utility functions
//-----------------------------------------------------------------------------

int compare(const void *, const void *);

void simplex_sort(simplex_t *);

void get_centroid(const simplex_t *, point_t *);

double modulus(double);

int continue_minimization(const simplex_t *, int, int, const optimset_t *);

void update_point(const simplex_t *, const point_t *, double, point_t *);

void copy_point(int, const point_t *, point_t *);

void swap_points(int, point_t *, point_t *);

#endif // NELDER_MEAD_H
