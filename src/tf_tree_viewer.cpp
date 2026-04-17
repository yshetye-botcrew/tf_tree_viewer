/**
 * tf_tree_viewer — DDS snapshot of a TF tree (tf2_msgs/TFMessage via message_manager / Fast DDS).
 *
 * What it does:
 *   - Subscribes to a static topic (default tf_static) and a dynamic topic (default tf).
 *   - For a configurable wall-clock window, merges all observed parent→child edges into a graph.
 *   - Prints an indented text tree (optionally rooted at --root).
 *   - Optionally emits Graphviz output with edge labels similar to ROS 2 tf2_tools view_frames
 *     (broadcaster label, average rate, buffer span, oldest/newest header stamp per edge).
 *
 * Implementation notes:
 *   - Topology uses header.frame_id as parent and child_frame_id as child (ROS transform convention).
 *   - Edge metrics use header.stamp only; broadcaster is not present on TransformStamped, so the
 *     DOT label uses the placeholder string "default_authority" to match typical view_frames PDFs.
 *   - Static-only edges (seen on tf_static but never on tf) use a high nominal rate and zero buffer
 *     length, consistent with view_frames for static transforms.
 *
 * Usage:
 *   tf_tree_viewer [options]
 *
 * Options (any order):
 *   --domain-id N          DDS domain id (default 0)
 *   --duration SEC         Collect transforms for SEC seconds (default 3.0)
 *   --static-topic NAME    tf_static topic (default: tf_static)
 *   --dynamic-topic NAME   tf topic (default: tf)
 *   --root FRAME           Print only subtree rooted at FRAME (default: all roots)
 *   --dot PATH             Write Graphviz .dot (edge labels as above)
 *   --xdot PATH            Write xdot via `dot -Txdot` (xdot viewer; requires Graphviz)
 *   --pdf PATH             Write PDF via `dot -Tpdf` (requires Graphviz)
 *   --rankdir LR|TB        Graph layout (default TB)
 *   --static-only          Exit after first non-empty static message (ignores duration)
 *
 * Topic names must match your Fast DDS discovery (often no leading "rt/"; match Foxglove / ros2 bridge).
 */

#include <message_manager/builtin_interfaces.hpp>
#include <message_manager/fastdds_transport.hpp>
#include <message_manager/publisher_options.hpp>
#include <message_manager/tf2_msgs.hpp>
#include <message_manager/type_id.hpp>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

// ---------------------------------------------------------------------------
// In-memory graph (thread-safe): topology + per-edge stamp statistics
// ---------------------------------------------------------------------------

/// Directed edge key: (parent_frame, child_frame).
using EdgeKey = std::pair<std::string, std::string>;

/// Aggregates all TransformStamped samples for one edge during the collection window.
struct EdgeStats {
    /// Min / max header.stamp (seconds, ROS time), over all samples on this edge.
    double min_stamp = std::numeric_limits<double>::infinity();
    double max_stamp = -std::numeric_limits<double>::infinity();
    /// Number of transform samples contributing to min/max (used for average rate).
    std::uint64_t n = 0;
    /// Whether this edge appeared on the static and/or dynamic subscription.
    bool seen_static = false;
    bool seen_dynamic = false;
};

struct Graph {
    std::mutex mtx;
    /// parent_frame -> child frames (set avoids duplicate edges from repeated messages).
    std::unordered_map<std::string, std::set<std::string>> children;
    /// Parallel to `children`: stamp stats for each (parent, child) pair.
    std::map<EdgeKey, EdgeStats> edge_stats;
    std::unordered_set<std::string> frames;
    std::atomic<int> static_msgs{0};
    std::atomic<int> dynamic_msgs{0};
};

/// ROS stamp as floating-point seconds (matches common tf2 / view_frames displays).
double stampToSec(const builtin_interfaces::msg::dds_::Time_& t) {
    return static_cast<double>(t.sec()) + static_cast<double>(t.nanosec()) * 1e-9;
}

/// Updates topology and per-edge stamp stats. Called from DDS subscriber callbacks (must stay cheap).
void mergeTf(const tf2_msgs::msg::dds_::TFMessage_& msg, Graph& g, bool is_static) {
    const auto& tfs = msg.transforms();
    if (tfs.empty()) return;

    std::lock_guard<std::mutex> lock(g.mtx);
    if (is_static)
        g.static_msgs.fetch_add(1, std::memory_order_relaxed);
    else
        g.dynamic_msgs.fetch_add(1, std::memory_order_relaxed);

    for (const auto& ts : tfs) {
        const std::string parent = ts.header().frame_id();
        const std::string child = ts.child_frame_id();
        if (parent.empty() || child.empty()) continue;
        g.frames.insert(parent);
        g.frames.insert(child);
        g.children[parent].insert(child);

        EdgeStats& st = g.edge_stats[EdgeKey{parent, child}];
        const double stamp_sec = stampToSec(ts.header().stamp());
        st.n++;
        st.min_stamp = std::min(st.min_stamp, stamp_sec);
        st.max_stamp = std::max(st.max_stamp, stamp_sec);
        if (is_static)
            st.seen_static = true;
        else
            st.seen_dynamic = true;
    }
}

// ---------------------------------------------------------------------------
// Graphviz: escaping, view_frames-style labels, DOT / PDF / xdot emission
// ---------------------------------------------------------------------------

/// Escape `"` and `\` inside a Graphviz double-quoted node name.
std::string dotEscape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 2);
    for (char c : s) {
        if (c == '\\' || c == '"')
            out.push_back('\\');
        out.push_back(c);
    }
    return out;
}

/// Nominal "average rate" shown by view_frames for static transforms (not computed from inter-arrival).
constexpr double kStaticAverageRate = 10000.0;

std::string formatDouble(double v, int prec) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(prec) << v;
    return oss.str();
}

/**
 * Multi-line edge label for DOT: mirrors the usual view_frames PDF fields.
 *
 * Dynamic edges: avg rate ≈ (n-1) / (t_max - t_min), buffer length = t_max - t_min.
 * Static-only edges: fixed high rate, zero buffer (see kStaticAverageRate).
 * Line breaks are emitted as the two-character sequence `\n` for Graphviz quoted strings.
 */
std::string edgeLabelViewFrames(const EdgeStats& st) {
    const bool static_edge = st.seen_static && !st.seen_dynamic;
    double avg_rate = 0.0;
    double buf_len = 0.0;
    double oldest = st.min_stamp;
    double newest = st.max_stamp;

    if (static_edge) {
        avg_rate = kStaticAverageRate;
        buf_len = 0.0;
    } else if (st.n >= 2 && newest > oldest) {
        avg_rate = static_cast<double>(st.n - 1) / (newest - oldest);
        buf_len = newest - oldest;
    } else if (st.n == 1) {
        avg_rate = 0.0;
        buf_len = 0.0;
    }

    std::ostringstream oss;
    oss << "Broadcaster: default_authority\\n";
    oss << "Average rate: " << formatDouble(avg_rate, 3) << "\\n";
    oss << "Buffer length: " << formatDouble(buf_len, 3) << "\\n";
    oss << "Most recent transform: " << formatDouble(newest, 6) << "\\n";
    oss << "Oldest transform: " << formatDouble(oldest, 6);
    return oss.str();
}

/// Writes a `digraph` with graph title (wall epoch) and one labeled edge per parent→child link.
void writeDot(std::ostream& out, const std::unordered_map<std::string, std::set<std::string>>& children,
              const std::map<EdgeKey, EdgeStats>& edge_stats, const std::string& rankdir, double recorded_epoch_sec) {
    out << "digraph tf_tree {\n";
    out << "  graph [labelloc=t, label=\"tf_tree_viewer Result\\n" << formatDouble(recorded_epoch_sec, 6)
        << "\", fontsize=14];\n";
    out << "  rankdir=" << rankdir << ";\n";
    out << "  node [shape=ellipse, style=filled, fillcolor=white];\n";
    out << "  edge [fontsize=9];\n";
    for (const auto& [parent, chset] : children) {
        for (const auto& child : chset) {
            out << "  \"" << dotEscape(parent) << "\" -> \"" << dotEscape(child) << "\" [label=\"";
            auto it = edge_stats.find(EdgeKey{parent, child});
            if (it != edge_stats.end())
                out << edgeLabelViewFrames(it->second);
            else
                out << "(no transform samples)";
            out << "\"];\n";
        }
    }
    out << "}\n";
}

/// Shell-safe single-quoted string for system("dot ...").
std::string shellSingleQuote(const std::string& s) {
    std::string out = "'";
    for (char c : s) {
        if (c == '\'')
            out += "'\\''";
        else
            out += c;
    }
    out += '\'';
    return out;
}

/**
 * Writes a temporary .dot file, invokes `dot <dot_format>`, removes the temp file.
 * `dot_format` is e.g. "-Tpdf" or "-Txdot". Requires the Graphviz `dot` executable on PATH.
 */
bool runDotFormat(const std::unordered_map<std::string, std::set<std::string>>& children,
                  const std::map<EdgeKey, EdgeStats>& edge_stats, const std::string& rankdir,
                  double recorded_epoch_sec, const char* dot_format, const std::string& out_path,
                  const std::string& tmp_suffix, std::string& err_out) {
    const std::string tmp_dot = out_path + tmp_suffix;
    {
        std::ofstream ofs(tmp_dot);
        if (!ofs) {
            err_out = "cannot open temp dot file: " + tmp_dot;
            return false;
        }
        writeDot(ofs, children, edge_stats, rankdir, recorded_epoch_sec);
    }
    const std::string cmd =
        std::string("dot ") + dot_format + " " + shellSingleQuote(tmp_dot) + " -o " + shellSingleQuote(out_path);
    const int st = std::system(cmd.c_str());
    std::remove(tmp_dot.c_str());
    if (st != 0) {
        err_out = std::string("`dot` failed (exit ") + std::to_string(st) + ", format " + dot_format +
                  "). Install Graphviz (e.g. sudo apt install graphviz) and ensure `dot` on PATH.";
        return false;
    }
    return true;
}

bool writePdfWithDot(const std::unordered_map<std::string, std::set<std::string>>& children,
                     const std::map<EdgeKey, EdgeStats>& edge_stats, const std::string& pdf_path,
                     const std::string& rankdir, double recorded_epoch_sec, std::string& err_out) {
    return runDotFormat(children, edge_stats, rankdir, recorded_epoch_sec, "-Tpdf", pdf_path,
                        ".tf_tree_viewer_tmp.dot", err_out);
}

bool writeXdotWithDot(const std::unordered_map<std::string, std::set<std::string>>& children,
                      const std::map<EdgeKey, EdgeStats>& edge_stats, const std::string& xdot_path,
                      const std::string& rankdir, double recorded_epoch_sec, std::string& err_out) {
    return runDotFormat(children, edge_stats, rankdir, recorded_epoch_sec, "-Txdot", xdot_path,
                        ".tf_tree_viewer_tmp_xdot.dot", err_out);
}

// ---------------------------------------------------------------------------
// Text tree: roots + DFS (cycle-safe)
// ---------------------------------------------------------------------------

/// Frames that never appear as a child of any edge (incoming degree 0).
std::vector<std::string> findRoots(const std::unordered_map<std::string, std::set<std::string>>& children,
                                    const std::unordered_set<std::string>& frames) {
    std::unordered_map<std::string, int> incoming;
    for (const auto& f : frames)
        incoming[f] = 0;
    for (const auto& [p, chs] : children) {
        for (const auto& c : chs)
            incoming[c]++;
    }
    std::vector<std::string> roots;
    for (const auto& [name, deg] : incoming) {
        if (deg == 0) roots.push_back(name);
    }
    std::sort(roots.begin(), roots.end());
    return roots;
}

/// Prints `node` and descendants under `children`, indenting by depth; `visiting` detects cycles.
void printDfs(std::ostream& out, const std::string& node, int depth,
              const std::unordered_map<std::string, std::set<std::string>>& children,
              std::unordered_set<std::string>& visiting) {
    if (visiting.count(node)) {
        out << std::string(static_cast<std::size_t>(depth) * 2u, ' ') << node << "  (cycle)\n";
        return;
    }
    visiting.insert(node);
    out << std::string(static_cast<std::size_t>(depth) * 2u, ' ') << node << "\n";
    auto it = children.find(node);
    if (it != children.end()) {
        std::vector<std::string> ch(it->second.begin(), it->second.end());
        for (const auto& c : ch)
            printDfs(out, c, depth + 1, children, visiting);
    }
    visiting.erase(node);
}

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

struct Args {
    int domain_id = 0;
    double duration_sec = 3.0;
    std::string topic_static = "tf_static";
    std::string topic_dynamic = "tf";
    std::string root_frame;
    std::string dot_path;
    std::string xdot_path;
    std::string pdf_path;
    /// Graphviz rankdir: TB = vertical (default), LR = horizontal
    std::string rankdir = "TB";
    bool static_only = false;
    bool help = false;
};

/// Minimal argv parser: flags with required arguments consume the next argv slot.
Args parseArgs(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--domain-id") == 0 && i + 1 < argc) {
            a.domain_id = std::atoi(argv[++i]);
        } else if (std::strcmp(argv[i], "--duration") == 0 && i + 1 < argc) {
            a.duration_sec = std::atof(argv[++i]);
        } else if (std::strcmp(argv[i], "--static-topic") == 0 && i + 1 < argc) {
            a.topic_static = argv[++i];
        } else if (std::strcmp(argv[i], "--dynamic-topic") == 0 && i + 1 < argc) {
            a.topic_dynamic = argv[++i];
        } else if (std::strcmp(argv[i], "--root") == 0 && i + 1 < argc) {
            a.root_frame = argv[++i];
        } else if (std::strcmp(argv[i], "--dot") == 0 && i + 1 < argc) {
            a.dot_path = argv[++i];
        } else if (std::strcmp(argv[i], "--xdot") == 0 && i + 1 < argc) {
            a.xdot_path = argv[++i];
        } else if (std::strcmp(argv[i], "--pdf") == 0 && i + 1 < argc) {
            a.pdf_path = argv[++i];
        } else if (std::strcmp(argv[i], "--rankdir") == 0 && i + 1 < argc) {
            std::string raw = argv[++i];
            std::string v = raw;
            for (auto& c : v)
                c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
            if (v == "LR" || v == "TB")
                a.rankdir = v;
            else
                std::cerr << "[tf_tree_viewer] Ignoring --rankdir \"" << raw << "\" (use LR or TB)\n";
        } else if (std::strcmp(argv[i], "--static-only") == 0) {
            a.static_only = true;
        } else if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0) {
            a.help = true;
            return a;
        }
    }
    return a;
}

void printHelp(const char* prog) {
    std::cerr
        << "Subscribe to DDS tf2_msgs/TFMessage, merge static + dynamic transforms, print TF tree.\n\n"
        << "Usage: " << prog << " [options]\n\n"
        << "  --domain-id N        DDS domain (default 0)\n"
        << "  --duration SEC       Collect time (default 3); ignored with --static-only\n"
        << "  --static-topic NAME  (default tf_static)\n"
        << "  --dynamic-topic NAME (default tf)\n"
        << "  --root FRAME         Print subtree from FRAME\n"
        << "  --dot PATH           Write Graphviz .dot (view_frames-style edge labels)\n"
        << "  --xdot PATH          Write xdot format (runs `dot -Txdot`; open with `xdot` app)\n"
        << "  --pdf PATH           Write PDF (runs `dot -Tpdf`; needs Graphviz)\n"
        << "  --rankdir LR|TB      Graphviz layout: LR=horizontal, TB=vertical (default TB)\n"
        << "  --static-only        Exit after first non-empty tf_static message\n\n"
        << "Visualize .dot / PDF:\n"
        << "  --pdf out.pdf        one-step PDF (like ros2 tf2_tools view_frames)\n"
        << "  dot -Tpng tf.dot -o tf.png && xdg-open tf.png\n"
        << "  dot -Tsvg tf.dot -o tf.svg\n"
        << "  xdot out.xdot   # or: xdot tf.dot (package xdot on Ubuntu/Debian)\n";
}

} // namespace

int main(int argc, char** argv) {
    // --- Parse flags ---
    Args args = parseArgs(argc, argv);
    if (args.help) {
        printHelp(argc >= 1 ? argv[0] : "tf_tree_viewer");
        return 0;
    }

    // --- DDS transport + TF subscribers (callbacks mutate `graph` under mutex) ---
    auto dds = std::make_shared<message_manager::FastDDSTransport>(
        message_manager::FastDDSTransport::Config{args.domain_id, "dds_tf_tree_viewer"});

    if (!dds->initialize()) {
        std::cerr << "[tf_tree_viewer] Failed to initialize Fast DDS\n";
        return 1;
    }

    Graph graph;

    auto cb_static = [&graph](const void* ptr, const std::type_index&) {
        mergeTf(*static_cast<const tf2_msgs::msg::dds_::TFMessage_*>(ptr), graph, true);
    };
    auto cb_dynamic = [&graph](const void* ptr, const std::type_index&) {
        mergeTf(*static_cast<const tf2_msgs::msg::dds_::TFMessage_*>(ptr), graph, false);
    };

    using TFMsg = tf2_msgs::msg::dds_::TFMessage_;
    auto sub_static = dds->registerSubscriber(
        args.topic_static, message_manager::type_id<TFMsg>, cb_static,
        message_manager::SubscriberOptions().transport<message_manager::FastDDSTransport>(
            {.qos_depth = 50, .transient_local = true, .reliable = true}));

    auto sub_dynamic = dds->registerSubscriber(
        args.topic_dynamic, message_manager::type_id<TFMsg>, cb_dynamic,
        message_manager::SubscriberOptions().transport<message_manager::FastDDSTransport>(
            {.qos_depth = 50, .transient_local = false, .reliable = true}));

    if (!sub_static || !sub_dynamic) {
        std::cerr << "[tf_tree_viewer] Failed to register subscribers (check topic names / domain)\n";
        dds->shutdown();
        return 1;
    }

    std::cout << "[tf_tree_viewer] domain=" << args.domain_id << "  static=\"" << args.topic_static << "\"  dynamic=\""
              << args.topic_dynamic << "\"\n";

    // --- Collection phase: either wait for first static data or run for --duration ---
    if (args.static_only) {
        std::cout << "[tf_tree_viewer] Waiting for first transforms on " << args.topic_static << " (max 15s)...\n";
        const auto t_wait = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - t_wait < std::chrono::seconds(15)) {
            {
                std::lock_guard<std::mutex> lock(graph.mtx);
                if (!graph.frames.empty()) break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    } else {
        std::cout << "[tf_tree_viewer] Collecting for " << args.duration_sec << " s (Ctrl+C to stop early)...\n";
        const auto t0 = std::chrono::steady_clock::now();
        while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < args.duration_sec) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    // Wall time at end of collection (Unix epoch seconds); stamped on DOT graph title like view_frames.
    const double recorded_epoch_sec = std::chrono::duration<double>(
                                            std::chrono::system_clock::now().time_since_epoch())
                                            .count();

    // --- Snapshot graph for output (avoid holding mutex while writing files / running dot) ---
    std::unordered_map<std::string, std::set<std::string>> children_copy;
    std::map<EdgeKey, EdgeStats> edge_stats_copy;
    std::unordered_set<std::string> frames_copy;
    int sm = 0, dm = 0;
    {
        std::lock_guard<std::mutex> lock(graph.mtx);
        children_copy = graph.children;
        edge_stats_copy = graph.edge_stats;
        frames_copy = graph.frames;
        sm = graph.static_msgs.load();
        dm = graph.dynamic_msgs.load();
    }

    std::cout << "[tf_tree_viewer] Received " << sm << " static message(s), " << dm << " dynamic message(s), "
              << frames_copy.size() << " frame(s), " << children_copy.size() << " parent link(s).\n";

    if (frames_copy.empty()) {
        std::cerr << "[tf_tree_viewer] No transforms — wrong topics or nothing publishing on this domain.\n";
        dds->shutdown();
        return 2;
    }

    // --- Console: optional single-root DFS, else every root ---
    std::cout << "\n--- TF tree (text) ---\n";
    if (!args.root_frame.empty()) {
        if (!frames_copy.count(args.root_frame)) {
            std::cerr << "[tf_tree_viewer] Warning: --root \"" << args.root_frame << "\" not in graph.\n";
        }
        std::unordered_set<std::string> visiting;
        printDfs(std::cout, args.root_frame, 0, children_copy, visiting);
    } else {
        const auto roots = findRoots(children_copy, frames_copy);
        if (roots.empty()) {
            std::cout << "(no roots found — possible cycle or empty children map; dumping frames)\n";
            std::vector<std::string> all(frames_copy.begin(), frames_copy.end());
            std::sort(all.begin(), all.end());
            for (const auto& f : all)
                std::cout << "  " << f << "\n";
        } else {
            for (const auto& r : roots) {
                std::cout << "\n[root: " << r << "]\n";
                std::unordered_set<std::string> visiting;
                printDfs(std::cout, r, 0, children_copy, visiting);
            }
        }
    }

    // --- Optional Graphviz outputs (DOT direct; PDF/xdot via `dot`) ---
    if (!args.dot_path.empty()) {
        std::ofstream ofs(args.dot_path);
        if (!ofs) {
            std::cerr << "[tf_tree_viewer] Failed to open --dot " << args.dot_path << "\n";
            dds->shutdown();
            return 3;
        }
        writeDot(ofs, children_copy, edge_stats_copy, args.rankdir, recorded_epoch_sec);
        ofs.close();
        std::cout << "\n[tf_tree_viewer] Wrote Graphviz: " << args.dot_path << "\n";
    }

    if (!args.xdot_path.empty()) {
        std::string err;
        if (!writeXdotWithDot(children_copy, edge_stats_copy, args.xdot_path, args.rankdir, recorded_epoch_sec, err)) {
            std::cerr << "[tf_tree_viewer] --xdot failed: " << err << "\n";
            dds->shutdown();
            return 4;
        }
        std::cout << "\n[tf_tree_viewer] Wrote xdot: " << args.xdot_path << " (via Graphviz dot -Txdot)\n";
    }

    if (!args.pdf_path.empty()) {
        std::string err;
        if (!writePdfWithDot(children_copy, edge_stats_copy, args.pdf_path, args.rankdir, recorded_epoch_sec, err)) {
            std::cerr << "[tf_tree_viewer] --pdf failed: " << err << "\n";
            dds->shutdown();
            return 5;
        }
        std::cout << "\n[tf_tree_viewer] Wrote PDF: " << args.pdf_path << " (via Graphviz dot -Tpdf)\n";
    }

    dds->shutdown();
    return 0;
}
