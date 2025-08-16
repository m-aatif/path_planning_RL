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

<p><em>Note: All changes so far maintain original algorithm logic - only terminology has been updated to reflect RL concepts.</em></p>