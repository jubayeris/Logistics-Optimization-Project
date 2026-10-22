# Logistics Optimization Project

A Python project that explores delivery route optimization using
Google OR-Tools and displays the resulting route on an interactive
map with Folium.

## Features

- Calculates great-circle distances between locations.
- Uses the OR-Tools routing solver to calculate a delivery route.
- Displays locations and the route on a map.
- Saves the map as `optimized_route.html`.

## Technologies

- Python
- Google OR-Tools
- NumPy
- geopy
- Folium

## Requirements

Install the dependencies with:

```bash
pip install -r requirements.txt
```

## Run the Project

```bash
python main.py
```

The current example uses four hard-coded locations in Seoul.
After a successful run, the script saves the map as
`optimized_route.html`.

## How It Works

1. Calculate distances between locations.
2. Create a vehicle routing model.
3. Configure a first-solution search strategy.
4. Solve the routing problem.
5. Visualize and save the route.

## Current Scope and Limitations

- The example uses one vehicle.
- Locations are defined in the source code.
- The objective uses straight-line geographic distance rather
  than actual road distance or traffic.
- The search strategy does not guarantee a globally optimal route.
- Delivery capacities and time windows are not configured.

## Author

Islam Md Jubayer
