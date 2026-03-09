#include "runtime/closed_loop_runtime.hpp"

#include <iostream>
#include <memory>

namespace {
std::unique_ptr<ClosedLoopRuntime> g_runtime;
}

extern "C" {

void sim_init(const mjModel* m, mjData* d) {
    try {
        g_runtime = std::make_unique<ClosedLoopRuntime>(m);
        g_runtime->initialize(m, d);
    } catch (const std::exception& e) {
        std::cerr << "[ClosedLoop] init failed: " << e.what() << std::endl;
        g_runtime.reset();
    }
}

void sim_step(const mjModel* m, mjData* d) {
    if (!g_runtime) {
        return;
    }
    g_runtime->step(m, d);
}

void sim_close() {
    g_runtime.reset();
}

}
