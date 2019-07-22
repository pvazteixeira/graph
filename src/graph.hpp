#pragma once

#include <algorithm>
#include <climits>
#include <fstream>
#include <map> // required for interfacing with multimap-based adjacency
#include <set>
#include <string>
#include <vector>

/**
 * \brief A lightweight graph class to support partitioning and segmentation
 * functionality.
 * \note This class could/should be templated to take other node types.
 */

class Graph {
public:
  Graph(){};

  ~Graph(){};

  Graph(const Graph &g)
      : nodes_(g.nodes_.begin(), g.nodes_.end()),
        edges_(g.edges_.begin(), g.edges_.end()) {}

  inline void addNode(const unsigned int &n) { nodes_.emplace(n); };

  inline void removeNode(const unsigned int &n) {
    nodes_.erase(n);
    for (auto eit = edges_.begin(); eit != edges_.end();) {
      if ((n == eit->first) || (n == eit->second)) {
        eit = edges_.erase(eit);
      } else {
        ++eit;
      }
    }
  };

  inline void removeNodes(const std::set<unsigned int> &nodes) {
    for (auto n : nodes) {
      removeNode(n);
    }
  }

  inline void addNodes(const std::set<unsigned int> &nodes) {
    nodes_.insert(nodes.begin(), nodes.end());
  }

  inline bool hasNode(const unsigned int &n) const {
    return (nodes_.find(n) != nodes_.end());
  }

  inline std::set<unsigned int> getNodes(void) const { return (nodes_); };

  inline unsigned int getNodeCount(void) const { return (nodes_.size()); };

  inline void addEdge(const std::pair<unsigned int, unsigned int> &e) {
    // add nodes if they do not exist.
    if (nodes_.find(e.first) == nodes_.end()) {
      nodes_.emplace(e.first);
    }
    if (nodes_.find(e.second) == nodes_.end()) {
      nodes_.emplace(e.second);
    }
    // ensure edge order: e1<e2 for edge (e1,e2)
    if (e.first < e.second) {
      edges_.emplace(e);
    } else if (e.first > e.second) {
      edges_.emplace(std::pair<unsigned int, unsigned int>(e.second, e.first));
    }
  };

  inline void addEdge(const unsigned int &n1, const unsigned int &n2) {
    addEdge(std::make_pair(n1, n2));
  }

  Graph(const std::set<std::pair<unsigned int, unsigned int>> &edges) {
    for (auto e : edges) {
      addEdge(e);
    }
  };

  Graph(const std::multimap<unsigned int, unsigned int> &adjacency) {
    for (auto e : adjacency) {
      addEdge(std::pair<unsigned int, unsigned int>(e.first, e.second));
    }
  };

  std::set<std::pair<unsigned int, unsigned int>>
  getEdges(const std::set<unsigned int> &nodes) const {
    std::set<std::pair<unsigned int, unsigned int>> edges;
    for (auto e : edges_) {
      if ((nodes.find(e.first) != nodes.end()) &&
          (nodes.find(e.second) != nodes.end())) {
        edges.emplace(e);
      }
    }
    return (edges);
  };

  std::set<std::pair<unsigned int, unsigned int>> getEdges(void) const {
    return (edges_);
  }

  inline unsigned int getFirstNode(void) const {
    unsigned int first = UINT_MAX;
    for (auto n : nodes_) {
      if (n < first) {
        first = n;
      }
    }
    return (first);
  };

  inline unsigned int getEdgeCount(void) const { return (edges_.size()); };

  std::set<unsigned int> getAdjacentNodes(const unsigned int &n) const {
    std::set<unsigned int> adjacent;
    if (nodes_.find(n) != nodes_.end()) {
      for (auto e : edges_) {
        if (n == e.first) {
          adjacent.emplace(e.second);
        } else if (n == e.second) {
          adjacent.emplace(e.first);
        }
      }
    }
    return (adjacent);
  };

  std::set<unsigned int>
  getAdjacentNodes(const std::set<unsigned int> &nodes) const {
    std::set<unsigned int> adjacent;
    for (unsigned int n : nodes) {
      auto adj = getAdjacentNodes(n);
      for (auto a : adj) {
        if (nodes.find(a) == nodes.end()) {
          adjacent.emplace(a);
        }
      }
      // adjacent.insert(a.begin(), a.end());
    }
    return (adjacent);
  };

  Graph getComponent(const unsigned int &seed) const {
    std::set<unsigned int> component;
    component.emplace(seed);

    // grow component
    while (true) {
      std::set<unsigned int> neighbors = getAdjacentNodes(component);
      if (!neighbors.size()) {
        break;
      }
      component.insert(neighbors.begin(), neighbors.end());
    }

    return (getSubgraph(component));
  };

  Graph getSubgraph(const std::set<unsigned int> &nodes) const {
    Graph g;
    g.addNodes(nodes);
    auto e = getEdges(nodes);
    g.edges_.insert(e.begin(), e.end());

    return (g);
  };

  std::vector<Graph> getConnectedComponents(void) const {
    std::vector<Graph> components;
    Graph g(edges_);

    while (g.getNodeCount()) {
      Graph c = getComponent(*g.nodes_.begin());
      components.push_back(c);
      g.removeNodes(c.nodes_);
    }
    return (components);
  };

  /**
   * \brief Save graph in graphviz format
   * \param [in] filename
   * \param [in,optional] sets - sets of nodes
   * This saves the graph in graphviz format while assigning a (non-unique)
   * color to the elements of each of the sets.
   */
  void saveGraph(
      const std::string &filename,
      const std::vector<std::set<unsigned int>> &sets =
          std::vector<std::set<unsigned int>>(),
      const std::set<std::pair<unsigned int, unsigned int>> &additional_edges =
          std::set<std::pair<unsigned int, unsigned int>>(),
      const unsigned int &colorset = 8) const {
    std::ofstream dotfile(filename);
    dotfile << "graph {\n";
    dotfile << "node [style=filled, colorscheme=\"paired" +
                   std::to_string(colorset) + "\", fixedsize=true,width=0.4]\n";

    for (auto e : edges_) {
      dotfile << e.first << " -- " << e.second << "\n";
    }

    // now write the colored nodes
    if (sets.size()) {
      // dotfile << "node[colorscheme=paired12]\n";
      for (unsigned int i = 0; i < sets.size(); ++i) {
        for (auto p : sets[i]) {
          dotfile << p << "[fillcolor=" << 1 + i % colorset << "]\n";
        }
      }
    }

    // now write the additional edges
    for (auto e : additional_edges) {
      dotfile << e.first << " -- " << e.second << " [color=red, penwidth=3]\n";
    }

    dotfile << "}\n";
    dotfile.close();
  };

  inline void saveEdges(const std::string &filename){
    std::ofstream csvfile(filename);
    for (auto e : edges_) {
      csvfile << e.first << ", " << e.second << "\n";
    }
    csvfile.close();
  }

  // TODO: a new version of savegraph that just saves the nodes in the partition
  // (vector of sets) to make visualizations simpler. Doing so would require
  // computing the union of the partitions so we can check if both nodes in an
  // edge are in that union before adding that edge to the graphviz file.

private:
  std::set<unsigned int> nodes_;
  std::set<std::pair<unsigned int, unsigned int>> edges_;
};

inline void saveConnectedComponents(const std::vector<Graph> &components,
                                    const std::string &filename) {
  std::ofstream dot_file(filename);
  dot_file << "graph {\n";
  for (auto c : components) {
    dot_file << "subgraph {\n";
    auto edges = c.getEdges();
    for (auto e : edges) {
      dot_file << e.first << " -- " << e.second << "\n";
    }
    dot_file << "}\n";
  }
  dot_file << "}\n";
  dot_file.close();
};
