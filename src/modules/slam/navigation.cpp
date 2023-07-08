#include "navigation.hpp"

#include <algorithm>
#include <iostream>
#include <thread>
#include <vector>

#include "coordinates.hpp"

using namespace std;

int index(int val, int *list) {
  int i;
  for (i = 0; i < 4; i++) {
    if (val == list[i]) {
      return i;
    }
  }

  /* if no match found */
  return -1;
}

array<array<int, MAX_SIZE>, 2> dijkstra(int G[MAX_SIZE][MAX_SIZE],
                                        int C[MAX_SIZE][4], int startnode,
                                        int endnode) {
  int cost[MAX_SIZE][MAX_SIZE], distance[MAX_SIZE], pred[MAX_SIZE];
  int visited[MAX_SIZE], count, mindistance, nextnode, i, j;
  int n = MAX_SIZE;

  /* replace  0 to inf */
  for (i = 0; i < n; i++) {
    for (j = 0; j < n; j++) {
      if (G[i][j] == 0 || G[i][j] == -1)
        cost[i][j] = INF;
      else
        cost[i][j] = G[i][j];
    }
  }

  /* first step and initial */
  for (i = 0; i < n; i++) {
    distance[i] = cost[startnode][i];
    pred[i] = startnode;
    visited[i] = 0;
  }
  distance[startnode] = 0;
  visited[startnode] = 1;
  count = 1;

  /* next steps */
  while (count < n - 1) {
    mindistance = INF;
    for (i = 0; i < n; i++)
      if (distance[i] < mindistance && !visited[i]) {
        mindistance = distance[i];
        nextnode = i;
      }
    visited[nextnode] = 1;

    for (i = 0; i < n; i++)
      if (!visited[i])
        if (mindistance + cost[nextnode][i] < distance[i]) {
          distance[i] = mindistance + cost[nextnode][i];
          pred[i] = nextnode;
        }
    count++;
  }

  /* print the result */
  int p = endnode;
  cout << "\nDistance of each node from the source node #" << startnode << endl;
  i = endnode;
  if (i != startnode) {
    cout << "\nDistance of node" << i << "=" << distance[i];
    cout << "\nPath=" << i;
    j = i;
    do {
      p = j;
      j = pred[j];
      cout << " <- " << j << "." << index(p, C[j]);

    } while (j != startnode);
    cout << endl;
  }

  array<int, MAX_SIZE> path_0 = {-1};
  array<int, MAX_SIZE> exits_0 = {-1};
  array<int, MAX_SIZE> path = {-1};
  array<int, MAX_SIZE> exits = {-1};
  fill(begin(path), end(path), -1);
  fill(begin(exits), end(exits), -1);
  fill(begin(path_0), end(path_0), -1);
  fill(begin(exits_0), end(exits_0), -1);

  int current_intersection = -1;
  int exit_intersection = -1;
  int previous_intersection = -1;
  int k = MAX_SIZE - 1;
  path_0[MAX_SIZE - 1] = endnode;
  exits_0[MAX_SIZE - 1] = -1;
  if (endnode != startnode) {
    current_intersection = endnode;
    do {
      previous_intersection = current_intersection;
      current_intersection = pred[current_intersection];
      exit_intersection = index(previous_intersection, C[current_intersection]);
      k -= 1;
      path_0[k] = current_intersection;
      exits_0[k] = exit_intersection;
    } while (current_intersection != startnode);
  }

  copy(begin(path_0) + k, end(path_0), begin(path));
  copy(begin(exits_0) + k, end(exits_0), begin(exits));

  array<array<int, MAX_SIZE>, 2> path_found = {path, exits};

  return path_found;

  // for (int i=0 ; i<max ; i++ )
  // {
  //    cout << path[i]<<"\t";
  // }
  // cout << endl;
  // for (int i=0 ; i<max ; i++ )
  // {
  //    cout << exits[i]<<"\t";
  // }
  // cout << endl;
}

Navigation::Navigation() {}

Navigation::~Navigation() {
  // No operation
}

int Navigation::exec_thread() {
  int intersection_count = 0;
  int turn_direction = 0;
  direction_t new_direction = UNKNOWN;
  int current_direction = -1;
  while (this->blackboard.running.get()) {
    if (!this->blackboard.navigation_enabled.get()) continue;

    start = this->blackboard.next_intersection.get();
    int destination = this->blackboard.destination.get();

    if (start == destination) {
      this->blackboard.destination_reached.set(true);
    }
    cout << end << ' ' << destination << endl;
    // new destination
    if (end != destination) {
      end = destination;
      path_found = dijkstra(this->G, this->C, this->start, this->end);
      intersection_count = 0;
    }
    // driving to destination
    else {
      // check if we passed a intersection
      if (path_found[0][intersection_count] ==
          blackboard.next_intersection.get()) {
        // get south/west/east/north direction
        int next_exit = path_found[1][intersection_count];

        intersection_count++;

        // get south/west ... which Robot is driving now
        current_direction = index(blackboard.last_intersection.get(), C[start]);
        new_direction =
            static_cast<direction_t>((next_exit - current_direction + 4) % 4);
        blackboard.direction.set(new_direction);

        // update next_intersection and last_intersection
        this->localization.track_map();
      }

      cout << " Navigation:  "
           << " Direction:" << new_direction
           << " first intersection: " << path_found[0][0]
           << " current direction: " << current_direction
           << " Next Intersction: " << start
           << " Last Intersection: " << blackboard.last_intersection.get()
           << endl;

      //      switch (current_direction) {
      //        case SOUTH_EXIT:
      //          turn_direction = current_direction;
      //          break;
      //        case WEST_EXIT:
      //          turn_direction = (current_direction + 3) % 4;
      //          break;
      //        case NORTH_EXIT:
      //          turn_direction = (current_direction + next_exit) % 4;
      //          break;
      //        case EAST_EXIT:
      //          current_direction = 3;
      //          break;
      //        default:
      //          cout << "Error: current_direction == -1" << endl;
      //          blackboard.running.set(false);
      //          return -1;
      //      }
      this_thread::sleep_for(chrono::milliseconds(100));
    }
  }

  return 0;
}
