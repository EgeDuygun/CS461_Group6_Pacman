using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;
using System.Threading;

public class PacMan : MonoBehaviour {

	public float speed = 1.0f;

	private Vector3 direction = Vector3.zero;

	private List<Vector3> pos_list = new List<Vector3>();

	private int action_index = 0;


	// Successor Code For BFS, DFS and A* Search //

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

	// Successor Algorithm for UCS (can go through walls at a larger cost)//

	public List<(Vector3, Vector3, float)> getsuccessor_ucs(Vector3 stat, List<Vector3> walls, List<Vector3> barriers) {

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

			if ((walls.Contains(new Vector3(nextx, nexty, 0)) == false) && (barriers.Contains(new Vector3(nextx, nexty, 0)) == false)) {
				var nextState = new Vector3(nextx, nexty, 0);
				float cost = 1;
				successors.Add((nextState, action, cost));
			} else if (barriers.Contains(new Vector3(nextx, nexty, 0)) == false) {
				var nextState = new Vector3(nextx, nexty, 0);
				float cost = 100;
				successors.Add((nextState, action, cost));
			}
		}
			return successors;  
	} 	


	// BFS ALGORITHM //
	public (List<Vector3>, HashSet<Vector3>) bf_search() {
		
		GameObject[] gos;
		gos = GameObject.FindGameObjectsWithTag("MazeWall");
		GameObject target = GameObject.FindGameObjectWithTag("TargetNode");

		Vector3 goal_node = target.transform.position;

		List<Vector3> wall_list = new List<Vector3>();   
		foreach (GameObject w in gos) {
			wall_list.Add(w.transform.position);
		}

		HashSet<Vector3> visited_nodes = new HashSet<Vector3>();
		List<Vector3> list_of_actions = new List<Vector3>();
		var myStack = new List<(Vector3, List<Vector3>)>();
		var starting_pos = transform.localPosition;
		myStack.Add((starting_pos, list_of_actions));


		Vector3 state;
		List<Vector3> path_to_follow;

		while (myStack.Count != 0) {
			state = myStack[myStack.Count-1].Item1;
			path_to_follow = myStack[myStack.Count-1].Item2;
			myStack.RemoveAt(myStack.Count-1);

			if (state.Equals(goal_node)) {
				list_of_actions = path_to_follow;
				visited_nodes.Add(state);
				return (list_of_actions, visited_nodes);
			}

			if (visited_nodes.Contains(state) == false){
				visited_nodes.Add(state);			
				foreach((Vector3, Vector3, float) node in getsuccessor(state, wall_list)) {
					List<Vector3> updated_path = new List<Vector3>();
					foreach(Vector3 ac in path_to_follow) {
						updated_path.Add(ac);
					}
					updated_path.Add(node.Item2);	
					myStack.Insert(0,(node.Item1, updated_path)); // BFS
				}
			} 
		}
		return (list_of_actions, visited_nodes);
	}
	// DFS ALGORITHM //
	public (List<Vector3>, HashSet<Vector3>) df_search() {
		
		GameObject[] gos;
		gos = GameObject.FindGameObjectsWithTag("MazeWall");
		GameObject target = GameObject.FindGameObjectWithTag("TargetNode");

		Vector3 goal_node = target.transform.position;

		List<Vector3> wall_list = new List<Vector3>();   
		foreach (GameObject w in gos) {
			wall_list.Add(w.transform.position);
		}

		HashSet<Vector3> visited_nodes = new HashSet<Vector3>();
		List<Vector3> list_of_actions = new List<Vector3>();
		var myStack = new List<(Vector3, List<Vector3>)>();
		var starting_pos = transform.localPosition;
		myStack.Add((starting_pos, list_of_actions));

		Vector3 state;
		List<Vector3> path_to_follow;

		while (myStack.Count != 0) {
			state = myStack[myStack.Count-1].Item1;
			path_to_follow = myStack[myStack.Count-1].Item2;

			myStack.RemoveAt(myStack.Count-1);

			if (state.Equals(goal_node)) {
				list_of_actions = path_to_follow;
				visited_nodes.Add(state);
				return (list_of_actions, visited_nodes);
			}

			if (visited_nodes.Contains(state) == false){
				visited_nodes.Add(state);				
				foreach((Vector3, Vector3, float) node in getsuccessor(state, wall_list)) {
					List<Vector3> updated_path = new List<Vector3>();
					foreach(Vector3 ac in path_to_follow) {
						updated_path.Add(ac);
					}
					updated_path.Add(node.Item2);	
					myStack.Add((node.Item1, updated_path)); // DFS
				}
			} 
		}
		return (list_of_actions, visited_nodes);
	}

	// UCS ALGORITHM //
	public (List<Vector3>, HashSet<Vector3>) ucs() {
		
		GameObject[] gos;
		gos = GameObject.FindGameObjectsWithTag("MazeWall");
		GameObject target = GameObject.FindGameObjectWithTag("TargetNode");

		GameObject[] gos2;
		gos2 = GameObject.FindGameObjectsWithTag("Barrier");

		List<Vector3> barrier_list = new List<Vector3>();   
		foreach (GameObject b in gos2) {
			barrier_list.Add(b.transform.position);
		}

		Vector3 goal_node = target.transform.position;

		List<Vector3> wall_list = new List<Vector3>();   
		foreach (GameObject w in gos) {
			wall_list.Add(w.transform.position);
		}

		HashSet<Vector3> visited_nodes = new HashSet<Vector3>();
		List<Vector3> list_of_actions = new List<Vector3>();
		var myStack = new List<(Vector3, List<Vector3>, float)>();
		var starting_pos = transform.localPosition;		
		myStack.Add((starting_pos, list_of_actions, 0));

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

			if (state.Equals(goal_node)) {
				list_of_actions = path_to_follow;
				visited_nodes.Add(state);
				return (list_of_actions, visited_nodes);
			}

			if (visited_nodes.Contains(state) == false){
				visited_nodes.Add(state);				
				foreach((Vector3, Vector3, float) node in getsuccessor_ucs(state, wall_list, barrier_list)) {
					List<Vector3> updated_path = new List<Vector3>();
					float updated_cost = new float();

					foreach(Vector3 ac in path_to_follow) {
						updated_path.Add(ac);
					}
					updated_path.Add(node.Item2);
					updated_cost = cost + node.Item3;
					myStack.Add((node.Item1, updated_path, updated_cost));
				}
			} 
		}
		return (list_of_actions, visited_nodes);
	}

	// A* SEARCH ALGORITHM (Manhattan Distance used as the heuristic) //
	public (List<Vector3>, HashSet<Vector3>) astar() {
		
		GameObject[] gos;
		gos = GameObject.FindGameObjectsWithTag("MazeWall");
		GameObject target = GameObject.FindGameObjectWithTag("TargetNode");


		Vector3 goal_node = target.transform.position;
		List<Vector3> wall_list = new List<Vector3>();   
		foreach (GameObject w in gos) {
			wall_list.Add(w.transform.position);
		}

		HashSet<Vector3> visited_nodes = new HashSet<Vector3>();
		List<Vector3> list_of_actions = new List<Vector3>();
		var myStack = new List<(Vector3, List<Vector3>, float)>();
		var starting_pos = transform.localPosition;		
		myStack.Add((starting_pos, list_of_actions, 0));

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

			if (state.Equals(goal_node)) {
				list_of_actions = path_to_follow;
				visited_nodes.Add(state);
				return (list_of_actions, visited_nodes);
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
					heuristic = System.Math.Abs(node.Item1.x - goal_node.x) + System.Math.Abs(node.Item1.y - goal_node.y);
					updated_path.Add(node.Item2);
					updated_cost = cost + node.Item3 + heuristic;
					myStack.Add((node.Item1, updated_path, updated_cost));
				}
			} 
		}
		return (list_of_actions, visited_nodes);

	}	

	private GameObject[] expanded;
	private float bright = 0;
	private List<Vector3> act_list;
	private List<Vector3> vis_nodes = new List<Vector3>();
	private bool founded = false;

	// Use this for initialization
	void Start () {	
		expanded = GameObject.FindGameObjectsWithTag("Expanded");
		Vector3 initial = transform.localPosition;
		var t1 = ucs();
		act_list = t1.Item1;
		vis_nodes = t1.Item2.ToList();

		foreach(Vector3 act in act_list) {
			//Debug.Log(list_of_actions.Count);
			//Debug.Log(act);			
			pos_list.Add(initial + act);
			initial = initial + act;
		}	
		//Debug.Log(vis_nodes.Count);
		//Debug.Log(act_list.Count);
	}
	


	// Update is called once per frame
	void Update () {
		
		if (founded == false) {
			//Debug.Log(vis_nodes[0]);
			if (vis_nodes[0] == pos_list[pos_list.Count - 1]) {
				founded = true;
				//Thread.Sleep(2000);
			}
			foreach (GameObject i in expanded) {				
				if (i.transform.position == vis_nodes[0]) {
					i.GetComponent<SpriteRenderer>().color = new Color(1, 1-(bright + 0.01f), 0,1);
					bright += 0.01f;
					vis_nodes.RemoveAt(0);
					break;
				}						
			}
			Thread.Sleep(25);
		} 
		else if (founded == true) {
			foreach (GameObject i in expanded) {				
				if (i.transform.position == transform.position) {
					i.GetComponent<SpriteRenderer>().color = new Color(0.3f, 1, 1, 1);
					break;
				}						
			}
			direction = act_list[action_index];
			UpdateOrientation ();
			transform.position = Vector3.MoveTowards(transform.position, pos_list[action_index], 0.1f);		
			if (transform.position == pos_list[action_index]) {
				action_index += 1;
			// OPTIONAL STARTS
				foreach (GameObject i in expanded) {				
				if (i.transform.position == transform.position) {
					i.GetComponent<SpriteRenderer>().color = new Color(0.3f, 1, 1, 1);
					break;
				}						
			}
			// OPTIONAL ENDS
			}
				
		}
		if (transform.position == pos_list[pos_list.Count -1]) {
			//Thread.Sleep(2000);
			Debug.Log("Time to quit!");
			Application.Quit();
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
