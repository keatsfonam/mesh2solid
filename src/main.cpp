#include "mesh2solid/pipeline.h"

#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace {

void print_usage() {
  std::cerr
      << "Usage:\n"
      << "  mesh2solid analyze <input-file> --out <dir> [--preset mechanical]"
      << " [--solid-threshold <0..1>]\n";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc < 2) {
      print_usage();
      return 1;
    }

    std::vector<std::string> args(argv + 1, argv + argc);
    if (args[0] == "--help" || args[0] == "-h") {
      print_usage();
      return 0;
    }

    if (args[0] != "analyze") {
      std::cerr << "Unknown command: " << args[0] << "\n";
      print_usage();
      return 1;
    }

    if (args.size() < 4) {
      print_usage();
      return 1;
    }

    mesh2solid::AnalyzeOptions options;
    options.input_path = args[1];

    for (std::size_t i = 2; i < args.size(); ++i) {
      const std::string& arg = args[i];
      if (arg == "--out" && i + 1 < args.size()) {
        options.output_dir = args[++i];
      } else if (arg == "--preset" && i + 1 < args.size()) {
        options.preset = args[++i];
      } else if (arg == "--solid-threshold" && i + 1 < args.size()) {
        options.solid_threshold = std::stod(args[++i]);
      } else {
        std::cerr << "Unexpected argument: " << arg << "\n";
        print_usage();
        return 1;
      }
    }

    if (options.output_dir.empty()) {
      std::cerr << "--out is required\n";
      print_usage();
      return 1;
    }

    if (options.solid_threshold < 0.0 || options.solid_threshold > 1.0) {
      std::cerr << "--solid-threshold must be between 0 and 1\n";
      return 1;
    }

    mesh2solid::RunReport report = mesh2solid::analyze(options);
    mesh2solid::write_outputs(options, report);

    std::cout << "Analyzed: " << options.input_path << "\n";
    std::cout << "Backend: " << report.backend << "\n";
    std::cout << "Plane regularization: " << report.plane_regularization.method;
    if (report.plane_regularization.applied) {
      std::cout << " (" << report.plane_regularization.adjusted_planes << " adjusted)";
    }
    std::cout << "\n";
    std::cout << "Method: "
              << mesh2solid::reconstruction_method_to_string(report.reconstruction.method) << "\n";
    std::cout << "Regions: " << report.regions.size() << "\n";
    std::cout << "Outcome: "
              << mesh2solid::reconstruction_outcome_to_string(report.reconstruction.outcome)
              << "\n";
    if (!report.reconstruction.failure_reasons.empty()) {
      std::cout << "Notes:\n";
      for (const auto& reason : report.reconstruction.failure_reasons) {
        std::cout << "  - " << reason << "\n";
      }
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "mesh2solid failed: " << error.what() << "\n";
    return 1;
  }
}
