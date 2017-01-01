The etasl_solver_qpoases package
============================

Introduction
------------

In eTaSL the task description in LUA is translated into a data-structure called *context*.  A *solver* translates
this context data-structure into a numerical problem that is then solved by a *numerical solver*.


The solver is now decoupled from the etasl_rtt component.
This component calls a solver for the qpOASES QP-solver. 

For more information on stack-of-tasks, see http://stack-of-tasks.github.io/

For more information on eTaSL, see https://people.mech.kuleuven.be/~eaertbel/etasl




Auther
------

Erwin Aertbeliën, 2016




