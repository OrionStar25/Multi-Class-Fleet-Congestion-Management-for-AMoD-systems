// max x0 + 2*x1 + 3*x2
// −x0 + x1 + x2 ≤ 20
// x0 − 3*x1 + x2 ≤ 30
// 0 ≤ x0 ≤ 40
// 0 ≤ x1 ≤ ∞
// 0 ≤ x2 ≤ ∞
// xi ∈ R


#include<ilcplex/ilocplex.h>
ILOSTLBEGIN

int main () 
{
	IloEnv env;
	IloModel model(env);
	IloNumVarArray x(env);

	x.add(IloNumVar(env, 0, 40));
	x.add(IloNumVar(env)); // default : between 0 and +∞
	x.add(IloNumVar(env));

	model.add(-x[0] + x[1] + x[2] <= 20);
	model.add(x[0] - 3 * x[1] + x[2] <= 30);
	model.add(IloMaximize(env, x[0] + 2 * x[1] + 3 * x[2]));

	IloCplex cplex(model);
	cplex.solve();

	cout<<"Max = "<<cplex.getObjValue()<<endl;
	env.end();
}