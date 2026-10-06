## Prerequisites
[Doxygen 1.12.0](https://github.com/doxygen/doxygen/releases/tag/Release_1_12_0) and [Graphviz](https://graphviz.org/download/) must first be installed. Both pages provide downloads for Windows, Linux and macOS.

To build the documentation, invoke:
````
doxygen
````

## Deployment

Documentation will automatically publish to GitHub Pages when:

* A release is made.
* A manual workflow dispatch is initiated and publishing is enabled.
* Any push to any branch is made when `CONTINUOUS_DOCUMENTATION` is enabled (intended for use in forks for development purposes).