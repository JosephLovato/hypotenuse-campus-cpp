# Hypotenuse Campus (C++)

Routing through open spaces using graph theory and image analysis techniques.

## Motivations

This project was initially motivated by a graph theory class in a graduate course I took at Colorado School of Mines. As a joke in college, I always told people that I "take the hypotenuse" across campus when I am in a hurry. In other words, I went off the sidewalks into the grass because, in general, the hypotenuse of a right triangle is less than the sum of it's two sides. I aimed to model the open space of campus as a graph, which could then be used in shortest path algorithms to generate these "hypotenuse" routes across campus. 

## Technical Notes

This repo contains a general template for implementing open space routing algorithms using a png image as the input map. In this sense, the distance is measured in pixels, which can be easily converted to real distances if the scale of the image is known. I used red pixels (where the 8-bit red value of a pixel is greater than 245) to represent innavigable space. I also include a special image where only green pixels (8-bit green value of a pixel is greater than 245) are navigable, representing the open space of campus restricted to only sidewalks, plazas, and clear walk ways. 

Currently, I have implemented two primary open-space to graph modeling algorithms:

- Spider: Each navigable pixel is a vertex and is connected to any of the navigable pixels just above, below, left, and right of it with a weight of $1$ and navigable inter-cardinal pixels with a weight of $\sqrt 2$. Each pixel can optionally be connected to any navigable pixels forming a 2-1 triangle (a chess knight's move) with a weight of $\sqrt 5$
  - I have built two flavors of this algorithm: one that uses the open paths and one that uses strict paths
- Smart: The spider graph inherently only allows hypotenuses at angles embedded in the pixel to pixel connections (for example, 45 deg). To resolve this issue, only corner pixels are made vertices, where we define a corner pixel to be a navigable pixel with exactly one adjacent innavigable pixel (of the 8 possible). This eliminates all pixels in the middle of open spaces and along straight walls, but does capture corners of buildings and any curved boundaries in the open space. The idea here is that these are the points in the open space that really matter when routing, and if give us true hypotenuses when connecting them. Bresenham’s line algorithm is used to determine if two points are "visible" to one another. If no innavigable pixels are encountered on the straight line between between two points, an edge is added between the respective vertices with a weight of the euclidean pixel-distance between the two points (Pythagorean). When routing, the start and end points are added to the graph with edges to any visible vertices.

The A* shortest-path routing algorithm is then used on both of these models to calculate the shortest distance between two points in the open space along with the route itself.

Timing has also been added to compare the efficiency of various methods.

## Presentation

You can find my most recent presentation on this project here: COMING SOON

## References

[1] A. Zingl, The Beauty of Bresenham’s Algorithm, https://zingl.github.io/bresenham.html. 

[2] Stefan Hahmann et al. Routing through open spaces – A performance comparison of algorithms. 2018. doi: 10.1080/10095020.2017.1399675. eprint: https://doi.org/10.1080/10095020.2017.1399675. url: https://doi.org/10.1080/10095020.2017.1399675.
