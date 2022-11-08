# Dynamic-Routing-in-Stochastic-Urban-Air-Mobility-Network-A-Markov-Decision-Process-Approach

Paper: Dynamic Routing in Stochastic Urban Air Mobility Network: A Markove Decision Process Approach

File 1: Austin_Dallas_Data  
Content: An excel file that contains 3 tables.   
   &emsp;      Sheet 1: Distance data for chosen city pairs.  
  &emsp;       Sheet 2: Origin setting of capacities and queue lengths of all nodes.   
    &emsp;     Sheet 3: Comparison setting of capacities and queue lengths of all nodes.   
         
File 2: Austin_Dallas_Code.m         
Content: A simulation code in Matlab for computing the safe strategies that minimize the total travel time from Dallas to Austin.    
Usage: Please download the MDP toolbox for Matlab (https://www.mathworks.com/matlabcentral/fileexchange/25786-markov-decision-processes-mdp-toolbox)      
     &emsp;    Variables can be changed, but please make sure they satisfies the Assumptions in the paper.   
      &emsp;   Run the script, and the policies produced will be the safe strategies that minimize the total travel time of the flight.   
