# Plain element of EUDM planner
int32 id
bool valid
bool risky
float32 final_cost
string sim_info

string lat_behaviors
string lon_behaviors

string lat_cmds
string lon_cmds

PlainCost[] stage_costs

PlainTrajectory ego_traj
PlainTrajectory[] surround_trajs