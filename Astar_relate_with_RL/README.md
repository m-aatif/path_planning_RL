# A* to Reinforcement Learning

This document details the variable renaming changes made to align the A* pathfinding implementation with Reinforcement Learning (RL) terminology while preserving the original algorithm's logic.

## Renamed Following Variable

<table border="1">
  <tr>
    <th><strong>Original A* Term</strong></th>
    <th><strong>RL Equivalent Term</strong></th>
    <th><strong>Rationale</strong></th>
  </tr>
  <tr>
    <td><code>start</code></td>
    <td><code>initial_state</code></td>
    <td>RL agents begin from initial state</td>
  </tr>
  <tr>
    <td><code>goal</code></td>
    <td><code>terminal_state</code></td>
    <td>RL uses terminal/absorbing states</td>
  </tr>
  <tr>
    <td><code>class Node</code></td>
    <td><code>class State</code></td>
    <td>Nodes → States in RL terminology</td>
  </tr>
  <tr>
    <td><code>open_set</code></td>
    <td><code>frontier_set</code></td>
    <td>RL explores state frontiers</td>
  </tr>
  <tr>
    <td><code>closed_set</code></td>
    <td><code>explored_set</code></td>
    <td>Tracks evaluated states</td>
  </tr>
  <tr>
    <td><code>cost</code></td>
    <td><code>cumulative_reward</code></td>
    <td>RL maximizes reward (negative cost)</td>
  </tr>
  <tr>
    <td><code>calc_heuristic()</code></td>
    <td><code>estimate_value()</code></td>
    <td>Heuristic ≈ Value estimate</td>
  </tr>
  <tr>
    <td><code>motion_model</code></td>
    <td><code>action_space</code></td>
    <td>Possible actions in RL</td>
  </tr>
  <tr>
    <td><code>calc_grid_index()</code></td>
    <td><code>discretize_state()</code></td>
    <td>Continuous→Discrete states</td>
  </tr>
  <tr>
    <td><code>verify_node()</code></td>
    <td><code>is_valid_state()</code></td>
    <td>State validity check</td>
  </tr>
</table>

## Conceptual Comparison

<table border="1">
  <tr>
    <th><strong>A* Concept</strong></th>
    <th><strong>RL Equivalent</strong></th>
    <th><strong>Key Differences</strong></th>
  </tr>
  <tr>
    <td><code>while 1:</code> loop</td>
    <td>Episode/Timestep loop</td>
    <td>In A*: Runs until path found<br>In RL: Runs until convergence</td>
  </tr>
  <tr>
    <td><code>open_set/closed_set</code></td>
    <td>State visitation records</td>
    <td>In A*: Tracks exploration<br>In RL: Tracks state values</td>
  </tr>
  <tr>
    <td><code>cost + heuristic</code></td>
    <td><code>reward + value_estimate</code></td>
    <td>In A*: Minimizes cost<br>In RL: Maximizes reward</td>
  </tr>
  <tr>
    <td><code>resolution</code> threshold</td>
    <td>Convergence threshold</td>
    <td>In A*: Uses fixed grid<br>In RL: Uses variable ε</td>
  </tr>
  <tr>
    <td>Obstacle checks</td>
    <td>Negative rewards</td>
    <td>In A*: Binary collision<br>In RL: Penalizes bad states</td>
  </tr>
</table>


# A* vs RL Formula Comparison

## Core Formulas

<table border="1">
  <tr>
    <th><strong>Component</strong></th>
    <th><strong>A* Algorithm</strong></th>
    <th><strong>Reinforcement Learning</strong></th>
    <th><strong>Relationship</strong></th>
  </tr>
  <tr>
    <td><strong>Total Cost/Value</strong></td>
    <td><code>F(n) = G(n) + H(n)</code></td>
    <td><code>V(s) = E[R + γV(s')]</code></td>
    <td>Both estimate future path quality</td>
  </tr>
  <tr>
    <td><strong>Path Cost (G)</strong></td>
    <td><code>G(n)</code>: Accumulated cost from start to node <code>n</code></td>
    <td><code>R</code>: Immediate reward after action</td>
    <td><code>G(n) ≈ -R</code> (negative because A* minimizes cost while RL maximizes reward)</td>
  </tr>
  <tr>
    <td><strong>Heuristic (H)</strong></td>
    <td><code>H(n)</code>: Estimated cost from <code>n</code> to goal (e.g., Euclidean distance)</td>
    <td><code>γV(s')</code>: Discounted future value</td>
    <td><code>H(n) ≈ V(s')</code> (both estimate remaining path quality)</td>
  </tr>
  <tr>
    <td><strong>Discount Factor</strong></td>
    <td>Not explicitly used</td>
    <td><code>γ</code>: Decays future rewards (0 ≤ γ ≤ 1)</td>
    <td>A* implicitly uses γ=1 (no discounting)</td>
  </tr>
</table>

## Key Differences in Calculation

<h4>A* (Deterministic)</h4>
<pre>
F(n) = G(n) + H(n)
Where:
- G(n) = ∑ motion_cost (exact known costs)
- H(n) = heuristic estimate (e.g., straight-line distance)
</pre>

<h4>RL (Stochastic)</h4>
<pre>
V(s) = E[R(s,a) + γ max V(s')]
Where:
- R(s,a) = immediate reward
- γ = discount factor for future rewards
- E[] = expected value over possible transitions
</pre>

## Mapping Table

<table border="1">
  <tr>
    <th><strong>A* Term</strong></th>
    <th><strong>RL Term</strong></th>
    <th><strong>Equivalent Role</strong></th>
  </tr>
  <tr>
    <td><code>F(n)</code> (Total cost)</td>
    <td><code>V(s)</code> (State value)</td>
    <td>Combined current+future estimate</td>
  </tr>
  <tr>
    <td><code>G(n)</code> (Path cost)</td>
    <td><code>-R</code> (Negative reward)</td>
    <td>Accumulated path metric</td>
  </tr>
  <tr>
    <td><code>H(n)</code> (Heuristic)</td>
    <td><code>γV(s')</code> (Future value)</td>
    <td>Estimate of remaining path quality</td>
  </tr>
</table>

<p><em>Note: All changes so far maintain original algorithm logic - only terminology has been updated to reflect RL concepts.</em></p>