#include <gsl/gsl_poly.h>
#include <gsl/gsl_block.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_interp.h>
#include <gsl/gsl_spline.h>
#include <gsl/gsl_poly.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_integration.h>
#include <vector>

const int prefered_degree = 3,overlap = 0;
#ifndef POLYNOMIAL
#define POLYNOMIAL

struct polynomial
{
    int deg;
    gsl_vector *nums;
};

polynomial poly(int deg);

polynomial poly_one();

polynomial poly_root(double root);

polynomial polyder(polynomial p);

polynomial poly_mult(polynomial a,polynomial b);
#endif

#ifndef SPLINE
#define SPLINE
class Spline
{
private:
    polynomial spl_poly;
    
    gsl_matrix *points;
    gsl_matrix *rotated_points;

    gsl_matrix *Q;
    gsl_vector *translation_vector;

    polynomial first_der;
    polynomial second_der;
    

    int path_id;
    int sort_index;
    double length;

public:
    polynomial get_SplPoly(){ return spl_poly;}
    void set_SplPoly(polynomial p){
        spl_poly.deg = p.deg;
        spl_poly.nums = p.nums;
        first_der = polyder(spl_poly);
        second_der = polyder(first_der);
    }

    gsl_matrix* get_points();
    void set_points(gsl_matrix *newpoints);

    gsl_matrix* get_rotated_points();
    void set_rotated_points(gsl_matrix *newpoints);

    gsl_matrix* get_Q();
    void set_Q(gsl_matrix *new_Q);

    gsl_vector* get_translation();
    void set_translation(gsl_vector *new_trans);

    int get_path_id();
    void set_path_id(int new_id);

    int get_sort_index();
    void set_sort_index(int new_sort);

    double get_length(){return length;}
    
    std::vector<float> interpolate(Spline spline,int number, std::pair<float,float> bounds = std::make_pair(-1,-1));
    



    Spline(polynomial interpolation_poly,gsl_matrix *points_mat,gsl_matrix *rotated,gsl_matrix *Q_mat, gsl_vector *translation,polynomial first, polynomial second, int path, int sort_ind);
    
    
    ~Spline();
};

#endif


std::vector<float> interpolate(Spline spline,int number, std::pair<float,float> bounds = std::make_pair(-1,-1));

gsl_matrix* rotation_matrix_gen(gsl_matrix *pnts);
gsl_vector *get_translation_vector(gsl_matrix *group);

gsl_matrix *transform_points(gsl_matrix *points, gsl_matrix *Q, gsl_vector *get_translation_vector);

gsl_matrix *reverse_transform(gsl_matrix *points, gsl_matrix *Q, gsl_vector *get_translation_vector);

polynomial lagrange_gen(gsl_matrix* points);

double arclength_f(double, void* params);

double arclength(polynomial poly, double x0,double x1);

std::pair<std::vector<Spline>,std::vector<int>> raceline_gen(gsl_matrix *res,int path_id,int points_per_spline = prefered_degree+1,bool loop = true);
