/*
 * diff_drive.cc
 *
 *  created on: 22.11.2023
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
#define sDIM 3
#define iDIM 2

/* data types for the ode solver */
typedef std::array<double,sDIM> state_type;
typedef std::array<double,iDIM> input_type;

/* sampling time */
const double tau = 0.3;
/* number of intermediate steps in the ode solver */
const int nint=5;
OdeSolver ode_solver(sDIM,nint,tau);

// SYSTEM ODE
/* we integrate the diff_drive ode by 0.3 sec (the result is stored in x)  */
const double rad = 0.06, L = 0.4; // roomba robot model specification
auto  diff_drive_post = [](state_type &x, input_type &u) -> void {

  /* the ode describing the differential drive robot dynamics */
  auto rhs =[](state_type& dx,  const state_type &x, input_type &u) {
    dx[0] = (rad/2.0)*(u[1]+u[0])*std::cos(x[2]);
    dx[1] = (rad/2.0)*(u[1]+u[0])*std::sin(x[2]);
    dx[2] = (rad/L)*(u[1]-u[0]);
  };
  ode_solver(rhs,x,u);
};

// GROWTH BOUND
/* computation of the growth bound (the result is stored in r)  */
auto radius_post = [](state_type &r, input_type &u) {
    double c = (rad/2.0)*std::abs(u[0]+u[1]);
    r[0] = r[0]+tau*c*r[2];
    r[1] = r[1]+tau*c*r[2];
};

/* forward declaration of the functions to setup the state space
 * input space and obstacles of the diff_drive example */
scots::SymbolicSet diff_driveCreateStateSpace(Cudd &mgr);
scots::SymbolicSet diff_driveCreateInputSpace(Cudd &mgr);

void diff_driveCreateObstacles(scots::SymbolicSet &obs);


int main() {
  /* to measure time */
  TicToc tt;
  /* there is one unique manager to organize the bdd variables */
  Cudd mgr;

  /****************************************************************************/
  /* construct SymbolicSet for the state space */
  /****************************************************************************/
  scots::SymbolicSet ss=diff_driveCreateStateSpace(mgr);
  ss.writeToFile("diff_drive_ss.bdd");

  /****************************************************************************/
  /* construct SymbolicSet for the obstacles */
  /****************************************************************************/
  /* first make a copy of the state space so that we obtain the grid
   * information in the new symbolic set */
  scots::SymbolicSet obs(ss);
  diff_driveCreateObstacles(obs);
  obs.writeToFile("diff_drive_obst.bdd");

  /****************************************************************************/
  /* we define the target set */
  /****************************************************************************/
  /* first make a copy of the state space so that we obtain the grid
   * information in the new symbolic set */
  scots::SymbolicSet ts(ss); // First target
  /* define the target set as a symbolic set */
  double H[4*sDIM]={-1, 0, 0,
                    1, 0, 0,
                    0,-1, 0,
                    0, 1, 0};

  /////////////////////////////NEW TARGETS////////////////////////////////////////
  /////////////////////////////NEW TARGETS////////////////////////////////////////
  /* compute inner approximation of P={ x | H x<= h1 }  */
  double h[4] = {-9.49,10,-3.09,3.9};
  ts.addPolytope(4,H,h, scots::INNER);
  ts.writeToFile("diff_drive_target1.bdd");

  scots::SymbolicSet ts_(ss); // second target
  /* define the target set as a symbolic set */
  double L[sDIM*sDIM]= {4,0,0, 
                        0,4,0,
                        0,0,0};
  double y[sDIM]={5,0.25,0};
  ts_.addEllipsoid(L,y, scots::OUTER);
  ts_.writeToFile("diff_drive_target2.bdd");

  /****************************************************************************/
  /* construct SymbolicSet for the input space */
  /****************************************************************************/
  scots::SymbolicSet is=diff_driveCreateInputSpace(mgr);

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
  abstraction.computeTransitionRelation(diff_drive_post, radius_post);
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
  BDD O = obs.getSymbolicSet();
  /**************************1st Target************************************/
  BDD T = ts.getSymbolicSet(); // first target
  tt.tic();
  /* compute controller */
  BDD C=fp.reachAvoid(T,O,1); // for first target
  tt.toc();

  /****************************************************************************/
  /* last we store the controller as a SymbolicSet 
   * the underlying uniform grid is given by the Cartesian product of 
   * the uniform gird of the space and uniform gird of the input space */
  /****************************************************************************/
  scots::SymbolicSet controller(ss,is);
  controller.setSymbolicSet(C);
  controller.writeToFile("diff_drive_controller1.bdd");
  std::cout << std::endl;

  /**************************2nd Target************************************/
  BDD T_ = ts_.getSymbolicSet(); // second target
  tt.tic();
  /* compute controller */
  BDD C_=fp.reachAvoid(T_,O,1);
  tt.toc();

  /****************************************************************************/
  /* last we store the controller as a SymbolicSet 
   * the underlying uniform grid is given by the Cartesian product of 
   * the uniform gird of the space and uniform gird of the input space */
  /****************************************************************************/
  scots::SymbolicSet controller_(ss,is);
  controller_.setSymbolicSet(C_);
  controller_.writeToFile("diff_drive_controller2.bdd");


  return 1;
}

scots::SymbolicSet diff_driveCreateStateSpace(Cudd &mgr) {

  /* setup the workspace of the synthesis problem and the uniform grid */
  /* lower bounds of the hyper rectangle */
  double lb[sDIM]={0,0,-M_PI-0.4};  
  /* upper bounds of the hyper rectangle */
  double ub[sDIM]={10,10,M_PI+0.4}; 
  /* grid node distance diameter */
  double eta[sDIM]={0.2,0.2,0.2};   


  scots::SymbolicSet ss(mgr,sDIM,lb,ub,eta);

  /* add the grid points to the SymbolicSet ss */
  ss.addGridPoints();

 return ss;
}

scots::SymbolicSet diff_driveCreateInputSpace(Cudd &mgr) {

  /* lower bounds of the hyper rectangle */
  double lb[iDIM]= {-1*(2.0/rad),-1*(2.0/rad)};  
  /* upper bounds of the hyper rectangle */
  double ub[iDIM]={1*(2.0/rad),1*(2.0/rad)}; 
  /* grid node distance diameter */
  double eta[iDIM]={0.3*(2.0/rad),0.3*(2.0/rad)};   

  scots::SymbolicSet is(mgr,iDIM,lb,ub,eta);
  is.addGridPoints();

  return is;
}


void diff_driveCreateObstacles(scots::SymbolicSet &obs) {

  /////////////////////////////NEW////////////////////////////////////////////////
  /////////////////////////////NEW////////////////////////////////////////////////
  /* add the obstacles to the symbolic set */
  /* the obstacles are defined as polytopes */
  /* define H* x <= h */

  double H[4*sDIM]={-1, 0, 0,
                    1, 0, 0,
                    0,-1, 0,
                    0, 1, 0};
  /* add outer approximation of P={ x | H x<= h1 } form state space */
  double h1[4] = {-0,4,-0,2};
  obs.addPolytope(4,H,h1, scots::OUTER);
  /* add outer approximation of P={ x | H x<= h2 } form state space */
  double h2[4] = {-0,2,-2,5};
  obs.addPolytope(4,H,h2, scots::OUTER);
  /* add outer approximation of P={ x | H x<= h3 } form state space */
  double h3[4] = {-3,5,-3.5,5};
  obs.addPolytope(4,H,h3, scots::OUTER); // tested INNER here
  /* add outer approximation of P={ x | H x<= h4 } form state space */
  double h4[4] = {-1,4,-7,8};
  obs.addPolytope(4,H,h4, scots::OUTER);
  /* add outer approximation of P={ x | H x<= h5 } form state space */
  double h5[4] = {-4,6,-6,7};
  obs.addPolytope(4,H,h5, scots::OUTER);
  /* add outer approximation of P={ x | |L (x-y)| <= 1 } form state space -> ellipsoid */ 
  double L1[sDIM*sDIM]= {2./3.,0,0, 
                        0,1,0,
                        0,0,0};
  double y1[sDIM]={7.5,8,0};
  obs.addEllipsoid(L1,y1, scots::INNER);
  /* add outer approximation of P={ x | H x<= h7 } form state space */
  double h7[4] = {-8,8.5,-4,6};
  obs.addPolytope(4,H,h7, scots::OUTER);
  /* add outer approximation of P={ x | H x<= h8 } form state space */
  double h8[4] = {-8,8.5,-0,3};
  obs.addPolytope(4,H,h8, scots::OUTER);
  /* add outer approximation of P={ x | H x<= h9 } form state space */
  double h9[4] = {-6,6.4,-0,6};
  obs.addPolytope(4,H,h9, scots::OUTER);
  /* add outer approximation of P={ x | H x<= h7 } form state space */
  double h10[4] = {-9.4,10,-4,6};
  obs.addPolytope(4,H,h10, scots::OUTER);
  /* add outer approximation of P={ x | H x<= h8 } form state space */
  double h11[4] = {-9.4,10,-0,3};
  obs.addPolytope(4,H,h11, scots::OUTER);
  /* add outer approximation of P={ x | H x<= h8 } form state space */
  double h12[4] = {-6.6,7.8,-6.4,7};
  obs.addPolytope(4,H,h12, scots::OUTER);
  /* add outer approximation of P={ x | H x<= h8 } form state space */
  double h13[4] = {-8.8,9.2,-6.6,7};
  obs.addPolytope(4,H,h13, scots::OUTER);
  /* add outer approximation of P={ x | H x<= h8 } form state space */
  double h14[4] = {-4.5,5.5,-8.5,9.5};
  obs.addPolytope(4,H,h14, scots::OUTER);
  /* add outer approximation of P={ x | H x<= h8 } form state space */
  double h15[4] = {-2,3,-8.5,9.5};
  obs.addPolytope(4,H,h15, scots::OUTER);
  /* add outer approximation of P={ x | |L (x-y)| <= 1 } form state space */
  double L2[sDIM*sDIM]= {2,0,0, 
                        0,2,0,
                        0,0,0};
  double y2[sDIM]={2.5,5.6,0};
  obs.addEllipsoid(L2,y2, scots::OUTER);


  /////////////////////////////OLD////////////////////////////////////////////////
  /////////////////////////////OLD////////////////////////////////////////////////
  /* add the obstacles to the symbolic set */
  /* the obstacles are defined as polytopes */
  /* define H* x <= h */
  
  // double H[4*sDIM]={-1, 0, 0,
  //                   1, 0, 0,
  //                   0,-1, 0,
  //                   0, 1, 0};
  // /* add outer approximation of P={ x | H x<= h1 } form state space */
  // double h1[4] = {-1,1.2,-0, 9};
  // obs.addPolytope(4,H,h1, scots::OUTER);
  // /* add outer approximation of P={ x | H x<= h2 } form state space */
  // double h2[4] = {-2.2,2.4,-0,5};
  // obs.addPolytope(4,H,h2, scots::OUTER);
  // /* add outer approximation of P={ x | H x<= h3 } form state space */
  // double h3[4] = {-2.2,2.4,-6,10};
  // obs.addPolytope(4,H,h3, scots::OUTER); // tested INNER here
  // /* add outer approximation of P={ x | H x<= h4 } form state space */
  // double h4[4] = {-3.4,3.6,-0,9};
  // obs.addPolytope(4,H,h4, scots::OUTER);
  // /* add outer approximation of P={ x | H x<= h5 } form state space */
  // double h5[4] = {-4.6 ,4.8,-1,10};
  // obs.addPolytope(4,H,h5, scots::OUTER);
  // /* add outer approximation of P={ x | H x<= h6 } form state space */
  // double h6[4] = {-5.8,6,-0,6};
  // obs.addPolytope(4,H,h6, scots::OUTER);
  // /* add outer approximation of P={ x | H x<= h7 } form state space */
  // double h7[4] = {-5.8,6,-7,10};
  // obs.addPolytope(4,H,h7, scots::OUTER);
  // /* add outer approximation of P={ x | H x<= h8 } form state space */
  // double h8[4] = {-7,7.2,-1,10};
  // obs.addPolytope(4,H,h8, scots::OUTER);
  // /* add outer approximation of P={ x | H x<= h9 } form state space */
  // double h9[4] = {-8.2,8.4,-0,8.5};
  // obs.addPolytope(4,H,h9, scots::OUTER);
  // /* add outer approximation of P={ x | H x<= h10 } form state space */
  // double h10[4] = {-8.4,9.3,-8.3,8.5};
  // obs.addPolytope(4,H,h10, scots::OUTER);
  // /* add outer approximation of P={ x | H x<= h11 } form state space */
  // double h11[4] = {-9.3,10,-7.1,7.3};
  // obs.addPolytope(4,H,h11, scots::OUTER);
  // /* add outer approximation of P={ x | H x<= h12 } form state space */
  // double h12[4] = {-8.4,9.3,-5.9,6.1};
  // obs.addPolytope(4,H,h12, scots::OUTER);
  // /* add outer approximation of P={ x | H x<= h13 } form state space */
  // double h13[4] = {-9.3,10 ,-4.7,4.9};
  // obs.addPolytope(4,H,h13, scots::OUTER);
  // /* add outer approximation of P={ x | H x<= h14 } form state space */
  // double h14[4] = {-8.4,9.3,-3.5,3.7};
  // obs.addPolytope(4,H,h14, scots::OUTER);
  // /* add outer approximation of P={ x | H x<= h15 } form state space */
  // double h15[4] = {-9.3,10 ,-2.3,2.5};
  // obs.addPolytope(4,H,h15, scots::OUTER);
  
}

