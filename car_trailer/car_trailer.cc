/*
 * car_trailer.cc - The abstraction size is relatively huge so only tested on small number of obstacles
 *
 *  created on: 05.12.2023
 *      author: mohammed adib oumer
 */

/*
 * information about this example is given in the readme file
 *
 */

#include <array>
#include <iostream>

#include "cuddObj.hh"

#include "SymbolicSet.hh"
#include "SymbolicModelGrowthBound.hh"

#include "TicToc.hh"
#include "RungeKutta4.hh"
#include "FixedPoint.hh"

#ifndef M_PI
#define M_PI 3.14159265359
#endif


/* state space dim */
#define sDIM 4
#define iDIM 2

/* data types for the ode solver */
typedef std::array<double,sDIM> state_type;
typedef std::array<double,iDIM> input_type;

/* sampling time */
const double tau = 0.3;
/* number of intermediate steps in the ode solver */
const int nint=5;
OdeSolver ode_solver(sDIM,nint,tau);

/* we integrate the car_trailer ode by 0.3 sec (the result is stored in x)  */
// car with trailer dynamics specifications. L = length between front and back wheels of car, d1 - distance from back wheels to trailer wheels
const double L = 0.1, d1 = 0.2;
auto  car_trailer_post = [](state_type &x, input_type &u) -> void {

  /* the ode describing the car_trailer */
  auto rhs =[](state_type& dx,  const state_type &x, input_type &u) {
      dx[0] = u[0]*std::cos(x[2]);
      dx[1] = u[0]*std::sin(x[2]);
      dx[2] = u[0]/L*std::tan(u[1]);
      dx[3] = u[0]/d1*std::sin(x[2]-x[3]);
  };
  ode_solver(rhs,x,u);
};

/* computation of the growth bound (the result is stored in r)  */
auto radius_post = [](state_type &r, input_type &u) {
    r[0] = r[0]+tau*r[2]*std::abs(u[0]);
    r[1] = r[1]+tau*r[2]*std::abs(u[0]);
    r[3] = r[3]+tau*(1.0/d1)*std::abs(u[0])*(r[2]+r[3]);
};

/* forward declaration of the functions to setup the state space
 * input space and obstacles of the car_trailer example */
scots::SymbolicSet car_trailerCreateStateSpace(Cudd &mgr);
scots::SymbolicSet car_trailerCreateInputSpace(Cudd &mgr);

void car_trailerCreateObstacles(scots::SymbolicSet &obs);


int main() {
  /* to measure time */
  TicToc tt;
  /* there is one unique manager to organize the bdd variables */
  Cudd mgr;

  /****************************************************************************/
  /* construct SymbolicSet for the state space */
  /****************************************************************************/
  scots::SymbolicSet ss=car_trailerCreateStateSpace(mgr);
  ss.writeToFile("car_trailer_ss.bdd");

  /****************************************************************************/
  /* construct SymbolicSet for the obstacles */
  /****************************************************************************/
  /* first make a copy of the state space so that we obtain the grid
   * information in the new symbolic set */
  scots::SymbolicSet obs(ss);
  car_trailerCreateObstacles(obs);
  obs.writeToFile("car_trailer_obst.bdd");

  /****************************************************************************/
  /* we define the target set */
  /****************************************************************************/
  /* first make a copy of the state space so that we obtain the grid
   * information in the new symbolic set */
  scots::SymbolicSet ts(ss);
  /* define the target set as a symbolic set */
  double H[4*sDIM]={-1, 0, 0, 0,
                    1, 0, 0, 0,
                    0,-1, 0, 0,
                    0, 1, 0, 0};
  /* compute inner approximation of P={ x | H x<= h1 }  */
  double h[4] = {-9,9.81,-0, .81};
  ts.addPolytope(4,H,h, scots::OUTER);
  ts.writeToFile("car_trailer_target.bdd");

  /****************************************************************************/
  /* construct SymbolicSet for the input space */
  /****************************************************************************/
  scots::SymbolicSet is=car_trailerCreateInputSpace(mgr);

  /****************************************************************************/
  /* setup class for symbolic model computation */
  /****************************************************************************/
  /* first create SymbolicSet of post variables 
   * by copying the SymbolicSet of the state space and assigning new BDD IDs */
  scots::SymbolicSet sspost(ss,1);
  /* instantiate the SymbolicModel */
  scots::SymbolicModelGrowthBound<state_type,input_type> abstraction(&ss, &is, &sspost);
  /* compute the transition relation */
  tt.tic();
  abstraction.computeTransitionRelation(car_trailer_post, radius_post);
  std::cout << std::endl;
  tt.toc();
  /* get the number of elements in the transition relation */
  std::cout << std::endl << "Number of elements in the transition relation: " << abstraction.getSize() << std::endl;

  /****************************************************************************/
  /* we continue with the controller synthesis */
  /****************************************************************************/
  /* we setup a fixed point object to compute reachabilty controller */
  scots::FixedPoint fp(&abstraction);
  /* the fixed point algorithm operates on the BDD directly */
  BDD T = ts.getSymbolicSet();
  BDD O = obs.getSymbolicSet();
  tt.tic();
  /* compute controller */
  BDD C=fp.reachAvoid(T,O,1);
  tt.toc();

  /****************************************************************************/
  /* last we store the controller as a SymbolicSet 
   * the underlying uniform grid is given by the Cartesian product of 
   * the uniform gird of the space and uniform gird of the input space */
  /****************************************************************************/
  scots::SymbolicSet controller(ss,is);
  controller.setSymbolicSet(C);
  controller.writeToFile("car_trailer_controller.bdd");

  return 1;
}

scots::SymbolicSet car_trailerCreateStateSpace(Cudd &mgr) {

  /* setup the workspace of the synthesis problem and the uniform grid */
  /* lower bounds of the hyper rectangle */
  double lb[sDIM]={0,0,-M_PI-0.4,-M_PI-0.4};  
  /* upper bounds of the hyper rectangle */
  double ub[sDIM]={10,10,M_PI+0.4,M_PI+0.4}; 
  /* grid node distance diameter */
  double eta[sDIM]={.4,.4,.2,.2};   


  scots::SymbolicSet ss(mgr,sDIM,lb,ub,eta);

  /* add the grid points to the SymbolicSet ss */
  ss.addGridPoints();

 return ss;
}

scots::SymbolicSet car_trailerCreateInputSpace(Cudd &mgr) {

  /* lower bounds of the hyper rectangle */  
  double lb[iDIM]={-2,-1}; // u - speed, psi(steering angle) - between -pi/2 and pi/2
  /* upper bounds of the hyper rectangle */
  double ub[iDIM]={2,1}; 
  /* grid node distance diameter */
  double eta[iDIM]={.5,.25};   

  scots::SymbolicSet is(mgr,iDIM,lb,ub,eta);
  is.addGridPoints();

  return is;
}

void car_trailerCreateObstacles(scots::SymbolicSet &obs) {

  /* add the obstacles to the symbolic set */
  /* the obstacles are defined as polytopes */
  /* define H* x <= h */
  double H[4*sDIM]={-1, 0, 0, 0,
                    1, 0, 0, 0,
                    0,-1, 0, 0,
                    0, 1, 0, 0};
  /* add outer approximation of P={ x | H x<= h1 } form state space */
  double h1[4] = {-1.8,4,-0, 2};
  obs.addPolytope(4,H,h1, scots::OUTER);
  /* add outer approximation of P={ x | H x<= h2 } form state space */
  double h2[4] = {-1.8,4,-4,10};
  obs.addPolytope(4,H,h2, scots::OUTER);
  /* add outer approximation of P={ x | H x<= h3 } form state space */
  double h3[4] = {-5.1,6,-6.8,7.2};
  obs.addPolytope(4,H,h3, scots::OUTER); // tested INNER here
  /* add outer approximation of P={ x | H x<= h4 } form state space */
  double h4[4] = {-6.4,8.2,-0,8};
  obs.addPolytope(4,H,h4, scots::OUTER);

}
