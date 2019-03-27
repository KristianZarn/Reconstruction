
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "nelder_mead.h"

//-----------------------------------------------------------------------------
// Main function
// - n is the dimension of the data
// - start is the initial point (unchanged in output)
// - solution is the minimizer
// - cost_function is a pointer to a fun_t type function
// - args are the optional arguments of cost_function
// - optimset are the optimisation settings
//-----------------------------------------------------------------------------

void nelder_mead(int n, const point_t *start, point_t *solution,
                 fun_t cost_function, const void *args,
                 const optimset_t *optimset) {
  // internal points
  point_t point_r;
  point_t point_e;
  point_t point_c;
  point_t centroid;

  // allocate memory for internal points
  point_r.x = (double*) malloc(n * sizeof(double));
  point_e.x = (double*) malloc(n * sizeof(double));
  point_c.x = (double*) malloc(n * sizeof(double));
  centroid.x = (double*) malloc(n * sizeof(double));

  int iter_count = 0;
  int eval_count = 0;

  // initial simplex has size n + 1 where n is the dimensionality pf the data
  simplex_t simplex;
  simplex.n = n;
  simplex.p = (point_t*) malloc((n + 1) * sizeof(point_t));
  for (int i = 0; i < n + 1; i++) {
    simplex.p[i].x = (double*) malloc(n * sizeof(double));
    for (int j = 0; j < n; j++) {
      simplex.p[i].x[j] =
          (i - 1 == j) ? (start->x[j] != 0.0 ? 1.05 * start->x[j] : 0.00025)
                       : start->x[j];
    }
    cost_function(n, simplex.p + i, args);
    eval_count++;
  }
  // sort points in the simplex so that simplex.p[0] is the point having
  // minimum fx and simplex.p[n] is the one having the maximum fx
  simplex_sort(&simplex);
  // ompute the simplex centroid
  get_centroid(&simplex, &centroid);
  iter_count++;

  // continue minimization until stop conditions are met
  while (continue_minimization(&simplex, eval_count, iter_count, optimset)) {
    int shrink = 0;

    if (optimset->verbose) {
      printf("Iteration %04d     ", iter_count);
    }
    update_point(&simplex, &centroid, RHO, &point_r);
    cost_function(n, &point_r, args);
    eval_count++;
    if (point_r.fx < simplex.p[0].fx) {
      update_point(&simplex, &centroid, RHO * CHI, &point_e);
      cost_function(n, &point_e, args);
      eval_count++;
      if (point_e.fx < point_r.fx) {
        // expand
        if (optimset->verbose) {
          printf("expand          ");
        }
        copy_point(n, &point_e, simplex.p + n);
      } else {
        // reflect
        if (optimset->verbose) {
          printf("reflect         ");
        }
        copy_point(n, &point_r, simplex.p + n);
      }
    } else {
      if (point_r.fx < simplex.p[n - 1].fx) {
        // reflect
        if (optimset->verbose) {
          printf("reflect         ");
        }
        copy_point(n, &point_r, simplex.p + n);
      } else {
        if (point_r.fx < simplex.p[n].fx) {
          update_point(&simplex, &centroid, RHO * GAMMA, &point_c);
          cost_function(n, &point_c, args);
          eval_count++;
          if (point_c.fx <= point_r.fx) {
            // contract outside
            if (optimset->verbose) {
              printf("contract out    ");
            }
            copy_point(n, &point_c, simplex.p + n);
          } else {
            // shrink
            if (optimset->verbose) {
              printf("shrink         ");
            }
            shrink = 1;
          }
        } else {
          update_point(&simplex, &centroid, -GAMMA, &point_c);
          cost_function(n, &point_c, args);
          eval_count++;
          if (point_c.fx <= simplex.p[n].fx) {
            // contract inside
            if (optimset->verbose) {
              printf("contract in     ");
            }
            copy_point(n, &point_c, simplex.p + n);
          } else {
            // shrink
            if (optimset->verbose) {
              printf("shrink         ");
            }
            shrink = 1;
          }
        }
      }
    }
    if (shrink) {
      for (int i = 1; i < n + 1; i++) {
        for (int j = 0; j < n; j++) {
          simplex.p[i].x[j] = simplex.p[0].x[j] +
                              SIGMA * (simplex.p[i].x[j] - simplex.p[0].x[j]);
        }
        cost_function(n, simplex.p + i, args);
        eval_count++;
      }
      simplex_sort(&simplex);
    } else {
      for (int i = n - 1; i >= 0 && simplex.p[i + 1].fx < simplex.p[i].fx; i--) {
        swap_points(n, simplex.p + (i + 1), simplex.p + i);
      }
    }
    get_centroid(&simplex, &centroid);
    iter_count++;
    if (optimset->verbose) {
      // print current minimum
      printf("[ ");
      for (int i = 0; i < n; i++) {
        printf("%.2f ", simplex.p[0].x[i]);
      }
      printf("]    %.2f \n", simplex.p[0].fx);
    }
  }

  // save solution in output argument
  solution->x = (double*) malloc(n * sizeof(double));
  copy_point(n, simplex.p + 0, solution);

  // free memory
  free(centroid.x);
  free(point_r.x);
  free(point_e.x);
  free(point_c.x);
  for (int i = 0; i < n + 1; i++) {
    free(simplex.p[i].x);
  }
  free(simplex.p);
}

//-----------------------------------------------------------------------------
// Simplex sorting
//-----------------------------------------------------------------------------

int compare(const void *arg1, const void *arg2) {
  const double fx1 = (((point_t *)arg1)->fx);
  const double fx2 = (((point_t *)arg2)->fx);

  if (fx1 == fx2) {
    return 0;
  } else {
    return (fx1 < fx2) ? -1 : 1;
  }
}

void simplex_sort(simplex_t *simplex) {
  qsort((void *)(simplex->p), simplex->n + 1, sizeof(point_t), compare);
}

//-----------------------------------------------------------------------------
// Get centroid (average position) of simplex
//-----------------------------------------------------------------------------

void get_centroid(const simplex_t *simplex, point_t *centroid) {
  for (int j = 0; j < simplex->n; j++) {
    centroid->x[j] = 0;
    for (int i = 0; i < simplex->n; i++) {
      centroid->x[j] += simplex->p[i].x[j];
    }
    centroid->x[j] /= simplex->n;
  }
}

//-----------------------------------------------------------------------------
// Asses if simplex satisfies the minimization requirements
//-----------------------------------------------------------------------------

int continue_minimization(const simplex_t *simplex, int eval_count,
                          int iter_count, const optimset_t *optimset) {
  if (eval_count > optimset->max_eval || iter_count > optimset->max_iter) {
    // stop if #evals or #iters are greater than the max allowed
    return 0;
  }
  double condx = -1.0;
  double condf = -1.0;
  for (int i = 1; i < simplex->n + 1; i++) {
    const double temp = fabs(simplex->p[0].fx - simplex->p[i].fx);
    if (condf < temp) {
      condf = temp;
    }
  }
  for (int i = 1; i < simplex->n + 1; i++) {
    for (int j = 0; j < simplex->n; j++) {
      const double temp = fabs(simplex->p[0].x[j] - simplex->p[i].x[j]);
      if (condx < temp) {
        condx = temp;
      }
    }
  }
  // continue if both tolx or tolf condition is not met
  return condx > optimset->tolx || condf > optimset->tolf;
}

//-----------------------------------------------------------------------------
// Update current point
//-----------------------------------------------------------------------------

void update_point(const simplex_t *simplex, const point_t *centroid,
                  double lambda, point_t *point) {
  const int n = simplex->n;
  for (int j = 0; j < n; j++) {
    point->x[j] = (1.0 + lambda) * centroid->x[j] - lambda * simplex->p[n].x[j];
  }
}

//-----------------------------------------------------------------------------
// Simple point_t manipulation utlities
//-----------------------------------------------------------------------------

void copy_point(int n, const point_t *src, point_t *dst) {
  for (int j = 0; j < n; j++) {
    dst->x[j] = src->x[j];
  }
  dst->fx = src->fx;
}

void swap_points(int n, point_t *p1, point_t *p2) {
  double temp;
  for (int j = 0; j < n; j++) {
    temp = p1->x[j];
    p1->x[j] = p2->x[j];
    p2->x[j] = temp;
  }
  temp = p1->fx;
  p1->fx = p2->fx;
  p2->fx = temp;
}
