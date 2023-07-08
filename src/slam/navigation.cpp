
#include "navigation.hpp"

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

array<array<int,max>,2> dijkstra(int G[max][max], int C[max][4], int startnode, int endnode) {
   int cost[max][max], distance[max], pred[max];
   int visited[max],count,mindistance,nextnode,i,j;
   int n = max;

   /* replace  0 to inf */
   for(i=0;i<n;i++) {
      for(j=0;j<n;j++) {
         if(G[i][j]==0 || G[i][j]==-1)
            cost[i][j]=INFINITY;
         else
            cost[i][j]=G[i][j];
      }
   }

   /* first step and initial */
   for(i=0;i<n;i++) {
      distance[i]=cost[startnode][i];
      pred[i]=startnode;
      visited[i]=0;
   }
   distance[startnode]=0;
   visited[startnode]=1;
   count=1;

   /* next steps */
   while(count < n-1) {
      mindistance=INFINITY;
      for(i=0;i<n;i++)
         if(distance[i] < mindistance && !visited[i]) {
         mindistance = distance[i];
         nextnode = i;
      }
      visited[nextnode]=1;

      for(i=0;i<n;i++)
         if(!visited[i])
            if(mindistance+cost[nextnode][i]<distance[i]) {
               distance[i]=mindistance+cost[nextnode][i];
               pred[i]=nextnode;
            }
      count++;
   }

   /* print the result */
   int p = endnode;
   cout<<"\nDistance of each node from the source node #"<< startnode << endl;
   i = endnode;
    if(i!=startnode) {
        cout<<"\nDistance of node"<<i<<"="<<distance[i];
        cout<<"\nPath="<<i;
        j = i;
        do {
        p = j;
        j = pred[j];
        cout<<" <- "<<j<<"."<<index(p, C[j]);

        }while(j!=startnode);
        cout<<endl;
    }
   
   int path_0[max] = {-1};
   int exits_0[max] = {-1};
   array<int,max> path = {-1};
   array<int,max> exits = {-1};
   fill_n(path, max, -1);
   fill_n(exits, max, -1);
   fill_n(path_0, max, -1);
   fill_n(exits_0, max, -1);


   int current_intersection = -1;
   int exit_intersection = -1;
   int previous_intersection = -1;
   int k = max-1;
   path_0[max-1] = endnode;
   exits_0[max-1] = -1;
   if(endnode != startnode) {
      current_intersection = endnode;
      do {
         previous_intersection = current_intersection;
         current_intersection = pred[current_intersection];
         exit_intersection = index(previous_intersection, C[current_intersection]);
         k -= 1;
         path_0[k] = current_intersection;
         exits_0[k] = exit_intersection;
      }while(current_intersection!=startnode);
   }

   copy(path_0+k, path_0+max, path);
   copy(exits_0+k, exits_0+max, exits);

   array<array<int,max>,2> path_found = {path, exits};

   return  path_found;

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

Navigation::Navigation() {
    
    auto path_found = dijkstra(this->G, this->C, this->start, this->end);
    this->blackboard.path_found.set(path_found);

}

Navigation::~Navigation() {
    // No operation
}

int Navigation::exec_thread(){
    while (this->blackboard.navigation_enabled.get()) {
        int start  = 0;
        /* code */
    }
    
    return 0;
}


