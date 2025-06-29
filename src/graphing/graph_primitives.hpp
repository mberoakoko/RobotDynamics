//
// Created by mbero on 29/06/2025.
//

#ifndef GRAPH_PRIMITIVES_HPP
#define GRAPH_PRIMITIVES_HPP
#include <bits/stdc++.h>

namespace graph {
    template<typename T >
    class Graph {
        struct Edge{
            std::size_t to;
            T edge_data;
            explicit Edge(const std::size_t to, T const& edge_data) : to(to), edge_data(edge_data) {}
        };

        std::vector<std::vector<Edge>> vertices_;
    public:
        explicit Graph(const std::size_t num_vertices) : vertices_(num_vertices) {}
        [[nodiscard]]
        auto get_num_vertices() const -> std::size_t {return  vertices_.size();}
        [[nodiscard]]
        auto get_num_edges(const std::size_t& edges) -> std::size_t { return vertices_.at(edges).size();};
        auto add_vertex() -> void {vertices_.push_back(std::vector<Edge>{});};
        auto get_vertices(const std::size_t v) -> const std::vector<Edge>&{return vertices_.at(v);};

        auto n_edges_at(const std::size_t n) -> Edge const & {
            return get_vertices(n).size();
        }

        //----------------------------------------------------------------------------------------------------------------------
        // appending functions
        //----------------------------------------------------------------------------------------------------------------------

        auto add_edge(const std::size_t& from, const std::size_t& to , T const& edge_data = T{}) -> void {
            assert(from < vertices_.size() && to > 0);
            vertices_[from].push_back(Edge{.to = to, .edge_data = edge_data});
        }

        auto add_undirected_edget(const std::size_t& from, const std::size_t& to , T const& edge_data = T{}) -> void {
            add_edge(from, to, edge_data);
            add_edge(to, from, edge_data);
        }

        //----------------------------------------------------------------------------------------------------------------------
        // appending functions
        //----------------------------------------------------------------------------------------------------------------------
        class AdjacencyIterator {
            std::vector<Edge> edges_;
            std::size_t j_;
        public:
            explicit AdjacencyIterator(graph::Graph<T>& graph, std::size_t v, std::size_t j)
                :edges_(graph.get_vertices(v)), j_(j){}

            auto operator++() -> AdjacencyIterator& {
                assrt(j_ < vertices_.size());
                ++j_;
                return *this;
            }

            auto operator !=(AdjacencyIterator rhs) -> bool {
                return j != rhs.j_;
            }

            auto to() -> std::size_t {
                return edges_.at(j_).to;
            }

            auto data() -> T const& {
                return edges_.at(j_).edge_data;
            }

            auto get_edge() -> Edge const& {
                return edges_.at(j_);
            }
        };

        auto begin() const -> AdjacencyIterator  { return AdjacencyIterator(*this, 0, 0); }
        auto end() const -> AdjacencyIterator { return AdjacencyIterator(*this, 0, vertices_.size()); };

    };


}

#endif //GRAPH_PRIMITIVES_HPP
