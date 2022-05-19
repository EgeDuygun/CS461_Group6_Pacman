using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class pacman_mini : MonoBehaviour {

	public float speed = 1.0f;

	private Vector3 direction = Vector3.zero;

	private List<Vector3> act_list = new List<Vector3>();

	private List<Vector3> pac_pos_list = new List<Vector3>();

	private List<Vector3> g1_pos_list = new List<Vector3>();

	private List<Vector3> g2_pos_list = new List<Vector3>();

	private int action_index = 0;

	// Successor Code for Minimax and Expectimax //

	public (Vector3, Vector3, Vector3, List<Vector3>, float) generate_successor(int agent_index, Vector3 action, Vector3 p_pos, Vector3 g1_pos, Vector3 g2_pos, List<Vector3> pellets, List<Vector3> walls, float score) {
		float x;
		float y;
		float time_penalty = 1;

		if (agent_index == 0) {
			x = p_pos.x;
			y = p_pos.y;
		}

		else if (agent_index == 1) {
			x = g1_pos.x;
			y = g1_pos.y;
		}

		else if (agent_index == 2) {
			x = g2_pos.x;
			y = g2_pos.y;
		}

		else {
			x = 0;
			y = 0;
			Debug.Log("Invalid agent index!");
			//return null;
		}		

        float dx = action.x;
        float dy = action.y;
        var nextx = x + dx;
        var nexty = y + dy;
		float main_score = score;
		
        if (walls.Contains(new Vector3(nextx, nexty, 0)) == false) {
            var nextState = new Vector3(nextx, nexty, 0);			
            if (agent_index == 0) {
				List<Vector3> main_pellets = new List<Vector3>(pellets);		
				if (nextState.Equals(g1_pos) || nextState.Equals(g2_pos)) {
					main_score = main_score - 500;
				}
				if (pellets.Contains(nextState)) {						
					main_score = main_score + 10;
					main_pellets.Remove(nextState);
				}
                return (nextState, g1_pos, g2_pos, main_pellets, main_score - time_penalty);
            }
            else if (agent_index == 1) {
                if (nextState.Equals(p_pos) || g2_pos.Equals(p_pos)) {
                    main_score = main_score - 500;
                }
                return (p_pos, nextState, g2_pos, pellets, main_score);
            }
            else {
                if (nextState.Equals(p_pos) || g1_pos.Equals(p_pos)) {
                    main_score = main_score - 500;
                }
                return (p_pos, g1_pos, nextState, pellets, main_score);
            }
        } else {
			return (new Vector3 (100,0,0),new Vector3 (0,100,0),new Vector3 (0,0,100), pellets, 0);
		}
    }
    

	// MINIMAX ALGORITHM

	public (Vector3, Vector3, Vector3, List<Vector3>, float) pacmanNextAction(Vector3 p_pos, Vector3 g1_pos, Vector3 g2_pos, List<Vector3> pellets, List<Vector3> walls, float score) {
		var legal_successors = new List<(Vector3, Vector3, Vector3, List<Vector3>, float)>();
        List<Vector3> actions = new List<Vector3>();
        actions.Add(Vector3.up);
        actions.Add(Vector3.down);
        actions.Add(Vector3.right);
        actions.Add(Vector3.left);

        foreach (Vector3 action in actions) {
			(Vector3, Vector3, Vector3, List<Vector3>, float) successor = better_generate_successor(0, action, p_pos, g1_pos, g2_pos, pellets, walls, score);
            if (successor.Item1.x != 100) {
                legal_successors.Add(successor);
            }
        }


        float max_utility = -1000000;
        int action_index = -1;
		float utility;
		
		foreach ((Vector3, Vector3, Vector3, List<Vector3>, float) succ in legal_successors) {
			utility = min_value(succ, 1, 0, walls);
			//Debug.Log(utility);
			//Debug.Log(succ.Item4.Count);
			if (utility > max_utility) {
				max_utility = utility;
				action_index = legal_successors.IndexOf(succ);
            }
        }		
        return legal_successors[action_index];
    }

	public float min_value((Vector3, Vector3, Vector3, List<Vector3>, float) suc, int agent_index, int depth, List<Vector3> walls) {
		if (depth >= 2 || suc.Item2.Equals(suc.Item1) || suc.Item3.Equals(suc.Item1) || suc.Item4.Count == 0) {
			return suc.Item5;
		}
 		var legal_successors = new List<(Vector3, Vector3, Vector3, List<Vector3>, float)>();
        List<Vector3> actions = new List<Vector3>();
        actions.Add(Vector3.up);
        actions.Add(Vector3.down);
        actions.Add(Vector3.right);
        actions.Add(Vector3.left);

        foreach (Vector3 action in actions) {
			(Vector3, Vector3, Vector3, List<Vector3>, float) successor = better_generate_successor(agent_index, action, suc.Item1, suc.Item2, suc.Item3, suc.Item4, walls, suc.Item5);
            if (successor.Item1.x != 100) {
                legal_successors.Add(successor);
            }
        }
		float v = 100000;
		foreach ((Vector3, Vector3, Vector3, List<Vector3>, float) succ in legal_successors) {
			if (agent_index >= 2) {
				v = System.Math.Min(v, max_value(succ, depth + 1, walls));
			}
			else {
				v = System.Math.Min(v, min_value(succ, agent_index + 1, depth, walls));
			}
		} 
		return v;
	}


	public float max_value((Vector3, Vector3, Vector3, List<Vector3>, float) suc, int depth, List<Vector3> walls) {
		if (depth >= 2 || suc.Item2.Equals(suc.Item1) || suc.Item3.Equals(suc.Item1) || suc.Item4.Count == 0) {
			return suc.Item5;
		}		
 		var legal_successors = new List<(Vector3, Vector3, Vector3, List<Vector3>, float)>();
        List<Vector3> actions = new List<Vector3>();
        actions.Add(Vector3.up);
        actions.Add(Vector3.down);
        actions.Add(Vector3.right);
        actions.Add(Vector3.left);

        foreach (Vector3 action in actions) {
			(Vector3, Vector3, Vector3, List<Vector3>, float) successor = better_generate_successor(0, action, suc.Item1, suc.Item2, suc.Item3, suc.Item4, walls, suc.Item5);
            if (successor.Item1.x != 100) {
                legal_successors.Add(successor);
            }
        }

		float v = -100000;
		foreach ((Vector3, Vector3, Vector3, List<Vector3>, float) succ in legal_successors) {
			v = System.Math.Max(v, min_value(succ, 1, depth, walls));
		} 
		return v;
	}




	// ALPHA BETA PRUNING MINIMAX

	public (Vector3, Vector3, Vector3, List<Vector3>, float) ab_pacmanNextAction(Vector3 p_pos, Vector3 g1_pos, Vector3 g2_pos, List<Vector3> pellets, List<Vector3> walls, float score) {
		var legal_successors = new List<(Vector3, Vector3, Vector3, List<Vector3>, float)>();
        List<Vector3> actions = new List<Vector3>();
        actions.Add(Vector3.up);
        actions.Add(Vector3.down);
        actions.Add(Vector3.right);
        actions.Add(Vector3.left);

        foreach (Vector3 action in actions) {
			(Vector3, Vector3, Vector3, List<Vector3>, float) successor = better_generate_successor(0, action, p_pos, g1_pos, g2_pos, pellets, walls, score);
            if (successor.Item1.x != 100) {
                legal_successors.Add(successor);
            }
        }


        float max_utility = -10000000;
        int action_index = -1;
		float utility;
		float alpha = -10000000;
		float beta = 10000000; 

		foreach ((Vector3, Vector3, Vector3, List<Vector3>, float) succ in legal_successors) {
			utility = ab_min_value(succ, 1, 0, walls, alpha, beta);
			//Debug.Log(utility);
			//Debug.Log(succ.Item4.Count);
			if (utility > max_utility) {
				max_utility = utility;
				action_index = legal_successors.IndexOf(succ);
				alpha = utility;
            }
        }		
        return legal_successors[action_index];
    }	
	
	public float ab_min_value((Vector3, Vector3, Vector3, List<Vector3>, float) suc, int agent_index, int depth, List<Vector3> walls, float alpha, float beta) {
		if (depth >= 2 || suc.Item2.Equals(suc.Item1) || suc.Item3.Equals(suc.Item1) || suc.Item4.Count == 0) {
			return suc.Item5;
		}
 		var legal_successors = new List<(Vector3, Vector3, Vector3, List<Vector3>, float)>();
        List<Vector3> actions = new List<Vector3>();
        actions.Add(Vector3.up);
        actions.Add(Vector3.down);
        actions.Add(Vector3.right);
        actions.Add(Vector3.left);

        foreach (Vector3 action in actions) {
			(Vector3, Vector3, Vector3, List<Vector3>, float) successor = better_generate_successor(agent_index, action, suc.Item1, suc.Item2, suc.Item3, suc.Item4, walls, suc.Item5);
            if (successor.Item1.x != 100) {
                legal_successors.Add(successor);
            }
        }
		float v = 10000000;
		foreach ((Vector3, Vector3, Vector3, List<Vector3>, float) succ in legal_successors) {
			if (agent_index >= 2) {
				v = System.Math.Min(v, ab_max_value(succ, depth + 1, walls, alpha, beta));
			}
			else {
				v = System.Math.Min(v, ab_min_value(succ, agent_index + 1, depth, walls, alpha, beta));
			}
			if (v < alpha) {
				return v;
			}
			beta = System.Math.Min(beta, v);			
		} 
		return v;
	}

	public float ab_max_value((Vector3, Vector3, Vector3, List<Vector3>, float) suc, int depth, List<Vector3> walls, float alpha, float beta) {
		if (depth >= 2 || suc.Item2.Equals(suc.Item1) || suc.Item3.Equals(suc.Item1) || suc.Item4.Count == 0) {
			return suc.Item5;
		}		
 		var legal_successors = new List<(Vector3, Vector3, Vector3, List<Vector3>, float)>();
        List<Vector3> actions = new List<Vector3>();
        actions.Add(Vector3.up);
        actions.Add(Vector3.down);
        actions.Add(Vector3.right);
        actions.Add(Vector3.left);

        foreach (Vector3 action in actions) {
			(Vector3, Vector3, Vector3, List<Vector3>, float) successor = better_generate_successor(0, action, suc.Item1, suc.Item2, suc.Item3, suc.Item4, walls, suc.Item5);
            if (successor.Item1.x != 100) {
                legal_successors.Add(successor);
            }
        }

		float v = -10000000;
		foreach ((Vector3, Vector3, Vector3, List<Vector3>, float) succ in legal_successors) {
			v = System.Math.Max(v, ab_min_value(succ, 1, depth, walls, alpha, beta));
			if (v > beta) {
				return v;
			}
			alpha = System.Math.Max(alpha, v);
		} 
		return v;
	}


	// EXPECTIMAX ALGORITHM

	public (Vector3, Vector3, Vector3, List<Vector3>, float) exp_pacmanNextAction(Vector3 p_pos, Vector3 g1_pos, Vector3 g2_pos, List<Vector3> pellets, List<Vector3> walls, float score) {
		var legal_successors = new List<(Vector3, Vector3, Vector3, List<Vector3>, float)>();
        List<Vector3> actions = new List<Vector3>();
        actions.Add(Vector3.up);
        actions.Add(Vector3.down);
        actions.Add(Vector3.right);
        actions.Add(Vector3.left);

        foreach (Vector3 action in actions) {
			(Vector3, Vector3, Vector3, List<Vector3>, float) successor = better_generate_successor(0, action, p_pos, g1_pos, g2_pos, pellets, walls, score);
            if (successor.Item1.x != 100) {
                legal_successors.Add(successor);
            }
        }

        float max_utility = -1000000;
        int action_index = -1;
		float utility;
		
		foreach ((Vector3, Vector3, Vector3, List<Vector3>, float) succ in legal_successors) {
			utility = exp_value(succ, 1, 0, walls);
			//Debug.Log(utility);
			//Debug.Log(succ.Item4.Count);
			if (utility > max_utility) {
				max_utility = utility;
				action_index = legal_successors.IndexOf(succ);
            }
        }		
        return legal_successors[action_index];
    }

	public float exp_value((Vector3, Vector3, Vector3, List<Vector3>, float) suc, int agent_index, int depth, List<Vector3> walls) {
		if (depth >= 2 || suc.Item2.Equals(suc.Item1) || suc.Item3.Equals(suc.Item1) || suc.Item4.Count == 0) {
			return suc.Item5;
		}
 		var legal_successors = new List<(Vector3, Vector3, Vector3, List<Vector3>, float)>();
        List<Vector3> actions = new List<Vector3>();
        actions.Add(Vector3.up);
        actions.Add(Vector3.down);
        actions.Add(Vector3.right);
        actions.Add(Vector3.left);

        foreach (Vector3 action in actions) {
			(Vector3, Vector3, Vector3, List<Vector3>, float) successor = better_generate_successor(agent_index, action, suc.Item1, suc.Item2, suc.Item3, suc.Item4, walls, suc.Item5);
            if (successor.Item1.x != 100) {
                legal_successors.Add(successor);
            }
        }
		float v = 100000;
		foreach ((Vector3, Vector3, Vector3, List<Vector3>, float) succ in legal_successors) {
			if (agent_index >= 2) {
				v = System.Math.Min(v, exp_max_value(succ, depth + 1, walls));
			}
			else {
				v = System.Math.Min(v, exp_value(succ, agent_index + 1, depth, walls));
			}
		} 
		return (v / legal_successors.Count);
	}


	public float exp_max_value((Vector3, Vector3, Vector3, List<Vector3>, float) suc, int depth, List<Vector3> walls) {
		if (depth >= 2 || suc.Item2.Equals(suc.Item1) || suc.Item3.Equals(suc.Item1) || suc.Item4.Count == 0) {
			return suc.Item5;
		}		
 		var legal_successors = new List<(Vector3, Vector3, Vector3, List<Vector3>, float)>();
        List<Vector3> actions = new List<Vector3>();
        actions.Add(Vector3.up);
        actions.Add(Vector3.down);
        actions.Add(Vector3.right);
        actions.Add(Vector3.left);

        foreach (Vector3 action in actions) {
			(Vector3, Vector3, Vector3, List<Vector3>, float) successor = better_generate_successor(0, action, suc.Item1, suc.Item2, suc.Item3, suc.Item4, walls, suc.Item5);
            if (successor.Item1.x != 100) {
                legal_successors.Add(successor);
            }
        }

		float v = -100000;
		foreach ((Vector3, Vector3, Vector3, List<Vector3>, float) succ in legal_successors) {
			v = System.Math.Max(v, exp_value(succ, 1, depth, walls));
		} 
		return v;
	}

	// GHOST MOVEMENTS

	public Vector3 ghostGreedyMove(Vector3 pacpos, Vector3 current_place, List<Vector3> walls) {
		
		Vector3 shortest = new Vector3();
		float min_dist = 10000;
		float dist;
		List<Vector3> actions = new List<Vector3>();
		actions.Add(Vector3.up);
		actions.Add(Vector3.down);
		actions.Add(Vector3.left);
		actions.Add(Vector3.right);

		float x = current_place.x;
		float y = current_place.y;

		foreach (Vector3 action in actions) {
			float dx = action.x;
			float dy = action.y;
			var nextx = x + dx;
			var nexty = y + dy;
			if (walls.Contains(new Vector3(nextx, nexty, 0)) == false) {
				dist = System.Math.Abs(nextx - pacpos.x) + System.Math.Abs(nexty - pacpos.y);
				if (dist < min_dist) {
					min_dist = dist;
					shortest = new Vector3(nextx,nexty,0);
				}
			}	
		}
		return shortest;
	}

	public Vector3 ghostRandomMove(Vector3 current_place, List<Vector3> walls, System.Random rn) {		
		
		List<Vector3> legals = new List<Vector3>();
		List<Vector3> actions = new List<Vector3>();
		actions.Add(Vector3.up);
		actions.Add(Vector3.down);
		actions.Add(Vector3.left);
		actions.Add(Vector3.right);

		float x = current_place.x;
		float y = current_place.y;

		foreach (Vector3 action in actions) {
			float dx = action.x;
			float dy = action.y;
			var nextx = x + dx;
			var nexty = y + dy;
			if (walls.Contains(new Vector3(nextx, nexty, 0)) == false) {
				var nextState = new Vector3(nextx, nexty, 0);
				legals.Add(nextState);
			}	
		}
		int rnd = rn.Next(0,legals.Count);
		return legals[rnd];
	}


	// USEFUL FUNCTIONS FOR BETTER EVALUATION FUNCTION

	public float closest_ghost(Vector3 p_pos, Vector3 g1_pos, Vector3 g2_pos) {
		float dist1 = System.Math.Abs(p_pos.x - g1_pos.x) + System.Math.Abs(p_pos.y - g1_pos.y);
		float dist2 = System.Math.Abs(p_pos.x - g2_pos.x) + System.Math.Abs(p_pos.y - g2_pos.y);
		return System.Math.Min(dist1, dist2);
	}

	public float closest_food(Vector3 p_pos, List<Vector3> pellet_list) {
		float min_dist = 10000;
		foreach(Vector3 food in pellet_list) {
			float dist = System.Math.Abs(p_pos.x - food.x) + System.Math.Abs(p_pos.y - food.y);
			if (dist < min_dist) {
				min_dist = dist;
			}
		}
		return min_dist;
	}

	public Vector3 maze_dist(Vector3 p_pos, List<Vector3> pellet_list) {
        Vector3 pellet = new Vector3(0,0,0);
		float min_dist = 10000;
		foreach(Vector3 food in pellet_list) {
			float dist = System.Math.Abs(p_pos.x - food.x) + System.Math.Abs(p_pos.y - food.y);
			if (dist < min_dist) {
				min_dist = dist;
                pellet = food;
			}
		}
		return pellet;
	}    

	public List<(Vector3, Vector3, float)> getsuccessor(Vector3 stat, List<Vector3> walls) {

		var successors = new List<(Vector3, Vector3, float)>();
		List<Vector3> actions = new List<Vector3>();
		actions.Add(Vector3.up);
		actions.Add(Vector3.down);
		actions.Add(Vector3.left);
		actions.Add(Vector3.right);

		foreach (Vector3 action in actions) {
			float x = stat.x;
			float y = stat.y;
			float dx = action.x;
			float dy = action.y;
			var nextx = x + dx;
			var nexty = y + dy;

			if (walls.Contains(new Vector3(nextx, nexty, 0)) == false) {
				var nextState = new Vector3(nextx, nexty, 0);
				float cost = 1;
				successors.Add((nextState, action, cost));
			}
		}
			return successors;  
	} 

	public float astar(Vector3 starting, Vector3 goal, List<Vector3> wall_list) {
		
		HashSet<Vector3> visited_nodes = new HashSet<Vector3>();
		List<Vector3> list_of_actions = new List<Vector3>();
		var myStack = new List<(Vector3, List<Vector3>, float)>();
		myStack.Add((starting, list_of_actions, 0));

		Vector3 state;
		List<Vector3> path_to_follow;
		float cost;
		(Vector3, List<Vector3>, float) priority;

		while (myStack.Count != 0) {
			priority = myStack[myStack.Count-1];
			foreach ((Vector3, List<Vector3>, float) path in myStack) {
				if (path.Item3 < priority.Item3) {
					priority = path;
				}
			}
			state = priority.Item1;
			path_to_follow = priority.Item2;
			cost = priority.Item3;

			myStack.Remove(priority);

			if (state.Equals(goal)) {
				list_of_actions = path_to_follow;
				visited_nodes.Add(state);
				return list_of_actions.Count;
			}

			if (visited_nodes.Contains(state) == false){
				visited_nodes.Add(state);				
				foreach((Vector3, Vector3, float) node in getsuccessor(state, wall_list)) {
					List<Vector3> updated_path = new List<Vector3>();
					float updated_cost = new float();
					float heuristic = new float();
					foreach(Vector3 ac in path_to_follow) {
						updated_path.Add(ac);
					}
					heuristic = System.Math.Abs(node.Item1.x - goal.x) + System.Math.Abs(node.Item1.y - goal.y);
					updated_path.Add(node.Item2);
					updated_cost = cost + node.Item3 + heuristic;
					myStack.Add((node.Item1, updated_path, updated_cost));
				}
			} 
		}
		return list_of_actions.Count;
	}	

	// BETTER EVALUATION FUNCTION

	public (Vector3, Vector3, Vector3, List<Vector3>, float) better_generate_successor(int agent_index, Vector3 action, Vector3 p_pos, Vector3 g1_pos, Vector3 g2_pos, List<Vector3> pellets, List<Vector3> walls, float score) {
		float x;
		float y;
		float time_penalty = 1;

		if (agent_index == 0) {
			x = p_pos.x;
			y = p_pos.y;
		}

		else if (agent_index == 1) {
			x = g1_pos.x;
			y = g1_pos.y;
		}

		else if (agent_index == 2) {
			x = g2_pos.x;
			y = g2_pos.y;
		}

		else {
			x = 0;
			y = 0;
			Debug.Log("Invalid agent index!");
			//return null;
		}		

        float dx = action.x;
        float dy = action.y;
        var nextx = x + dx;
        var nexty = y + dy;
		float main_score = score;
		
        if (walls.Contains(new Vector3(nextx, nexty, 0)) == false) {
            var nextState = new Vector3(nextx, nexty, 0);			
            if (agent_index == 0) {
				List<Vector3> main_pellets = new List<Vector3>(pellets);
				float close_ghost = closest_ghost(nextState, g1_pos, g2_pos);
				var close_food = maze_dist(nextState, main_pellets);	
                float maze_distance = astar(nextState, close_food, walls);
				//float negatives = (1/(close_ghost + 1)) + 10*(main_pellets.Count);
				//float positives = (100/(maze_distance + 0.001f));
				float negatives = (1/(close_ghost + 0.00002f)) + 100*(main_pellets.Count);
				float positives = (10/(maze_distance + 0.01f));
				score = (positives - negatives);
						
				if (nextState.Equals(g1_pos) || nextState.Equals(g2_pos)) {
					main_score = main_score - 500;
				}
				if (pellets.Contains(nextState)) {						
					main_score = main_score + 10;
					main_pellets.Remove(nextState);
				}
                return (nextState, g1_pos, g2_pos, main_pellets, score - time_penalty);
            }
            else if (agent_index == 1) {
                if (nextState.Equals(p_pos) || g2_pos.Equals(p_pos)) {
                    main_score = main_score - 500;
                }
                return (p_pos, nextState, g2_pos, pellets, main_score);
            }
            else {
                if (nextState.Equals(p_pos) || g1_pos.Equals(p_pos)) {
                    main_score = main_score - 500;
                }
                return (p_pos, g1_pos, nextState, pellets, main_score);
            }
        } else {
			return (new Vector3 (100,0,0),new Vector3 (0,100,0),new Vector3 (0,0,100), pellets, 0);
		}
    }


	private GameObject[] target;
	private List<Vector3> pellet_list = new List<Vector3>();  
	private List<Vector3> pellet_list2 = new List<Vector3>();  
	private int idx;
	private System.Random rng = new System.Random();


	// Use this for initialization
	void Start () {	

		GameObject[] gos;
		gos = GameObject.FindGameObjectsWithTag("MazeWall");
		target = GameObject.FindGameObjectsWithTag("TargetNode");
		List<Vector3> wall_list = new List<Vector3>();   

		foreach (GameObject w in gos) {
			wall_list.Add(w.transform.position);
		}

		foreach (GameObject p in target) {
			pellet_list2.Add(p.transform.position);
			pellet_list.Add(p.transform.position);
		}
		
		Vector3 p_pos = GameObject.FindGameObjectWithTag("MainAgent").transform.position;
		Vector3 g1_pos = GameObject.FindGameObjectWithTag("GhostOne").transform.position;
		Vector3 g2_pos = GameObject.FindGameObjectWithTag("GhostTwo").transform.position;
		float score = 0;
		(Vector3, Vector3, Vector3, List<Vector3>, float) pacman_action;
		int counter = 0;

		while (true) {
			if (counter == 500 || p_pos.Equals(g1_pos) || p_pos.Equals(g2_pos) || pellet_list.Count == 0) {
				if (p_pos.Equals(g1_pos) || p_pos.Equals(g2_pos)) {
					Debug.Log((pellet_list2.Count - pellet_list.Count)*10 - counter - 500);
				} else {
				Debug.Log((pellet_list2.Count - pellet_list.Count)*10 - counter);
				}
				break;
			}
			counter = counter + 1;
			pacman_action = exp_pacmanNextAction(p_pos, g1_pos, g2_pos, pellet_list, wall_list, score);
			pac_pos_list.Add(pacman_action.Item1);
			act_list.Add(pacman_action.Item1 - p_pos);
			score = pacman_action.Item5;
			pellet_list = pacman_action.Item4;
			p_pos = pacman_action.Item1;
			//Debug.Log(pacman_action);
			if (counter == 500 || p_pos.Equals(g1_pos) || p_pos.Equals(g2_pos) || pellet_list.Count == 0) {
				if (p_pos.Equals(g1_pos) || p_pos.Equals(g2_pos)) {
					Debug.Log((pellet_list2.Count - pellet_list.Count)*10 - counter - 500);
				} else {
				Debug.Log((pellet_list2.Count - pellet_list.Count)*10 - counter);
				}
				break;
			}	
			//g1_pos = ghostGreedyMove(p_pos, g1_pos, wall_list);
			//g2_pos = ghostGreedyMove(p_pos, g2_pos, wall_list);	
			g1_pos = ghostRandomMove(g1_pos, wall_list, rng);
			g2_pos = ghostRandomMove(g2_pos, wall_list, rng);	
			g1_pos_list.Add(g1_pos);
			g2_pos_list.Add(g2_pos);
			//Debug.Log((pellet_list2.Count - pellet_list.Count)*10 - counter);
		}
		//Debug.Log((pellet_list2.Count - pellet_list.Count)*10 - counter);
	}

	// Update is called once per frame

	void Update () {
		//Debug.Log(transform.position);
		direction = act_list[action_index];
		UpdateOrientation ();
		
		if (pellet_list2.Contains(transform.position)) {
			idx = pellet_list2.IndexOf(transform.position);
			//Debug.Log(transform.position);
			foreach (GameObject i in target) {				
				if (i.transform.position == transform.position) {
					i.GetComponent<SpriteRenderer>().color = new Color(0, 0, 0, 0);
					pellet_list2.RemoveAt(idx);
					break;
				}						
			}
		}
		
		transform.position = Vector3.MoveTowards(transform.position, pac_pos_list[action_index], 0.1f);

		GameObject.FindWithTag("GhostOne").transform.position = Vector3.MoveTowards(GameObject.FindWithTag("GhostOne").transform.position, g1_pos_list[action_index], 0.1f);

		GameObject.FindWithTag("GhostTwo").transform.position = Vector3.MoveTowards(GameObject.FindWithTag("GhostTwo").transform.position, g2_pos_list[action_index], 0.1f);


		if (transform.position.Equals(pac_pos_list[action_index])) {			
			action_index += 1;
		}		
		
	}


	void UpdateOrientation () {

		if (direction == Vector3.left) {

			transform.localScale = new Vector3 (-1, 1, 1);
			transform.localRotation = Quaternion.Euler (0, 0, 0);

		} else if (direction == Vector3.right) {

			transform.localScale = new Vector3 (1, 1, 1);
			transform.localRotation = Quaternion.Euler (0, 0, 0);

		} else if (direction == Vector3.up) {

			transform.localScale = new Vector3 (1, 1, 1);
			transform.localRotation = Quaternion.Euler (0, 0, 90);

		} else if (direction == Vector3.down) {

			transform.localScale = new Vector3 (1, 1, 1);
			transform.localRotation = Quaternion.Euler (0, 0, 270);
		}
	}
}




